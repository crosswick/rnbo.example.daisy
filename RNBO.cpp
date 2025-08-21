#include "daisy_pod.h"
#include "tlsf.h"
#include "ff.h" // FatFS

#define RNBO_USE_FLOAT32
#define RNBO_NOTHROW
#define RNBO_FIXEDLISTSIZE 64
#define RNBO_USECUSTOMALLOCATOR

#include "export/rnbo_source.h"

// implement our custom allocation methods, which are just re-directing all calls to
// a simple pool based allocator

tlsf_t myPool;

// SDRAM layout for Daisy: 64 MB total at 0xC0000000.
// Previously we consumed the full 64 MB for the RNBO allocator, which can
// starve other SDRAM users (e.g. streaming buffers) and risk overlaps with
// DSY_SDRAM_BSS. Reserve a small low region and place the pool above it.
#define SDRAM_BASE         (0xC0000000u)
#define SDRAM_SIZE_MB      (64u)
// Reserve N MB at the low end for other components (FatFS scratch, audio buffers, etc.).
// 8 MB is ample for typical streaming; adjust if your project needs more/less.
#ifndef RNBO_SDRAM_RESERVE_MB
#define RNBO_SDRAM_RESERVE_MB (8u)
#endif
#define POOLSIZE_MB        (SDRAM_SIZE_MB - RNBO_SDRAM_RESERVE_MB)
#define POOLSIZE           (POOLSIZE_MB * 1024u * 1024u)
#define POOLBASE           (SDRAM_BASE + (RNBO_SDRAM_RESERVE_MB * 1024u * 1024u))

namespace RNBO
{
	namespace Platform
	{

		void *malloc(size_t size)
		{
			return tlsf_malloc(myPool, size);
		}

		void free(void *ptr)
		{
			tlsf_free(myPool, ptr);
		}

		void *realloc(void *ptr, size_t size)
		{
			return tlsf_realloc(myPool, ptr, size);
		}

		void *calloc(size_t count, size_t size)
		{
			auto mem = malloc(count * size);
			memset(mem, 0, count * size);
			return mem;
		}
	}
}

using namespace daisy;

static const size_t vectorsize = 128;

DaisyPod hw;
MidiUsbHandler midi;

SdmmcHandler   sdcard;
FatFSInterface fsi;

RNBO::rnbomatic<> rnbo;

// CPU load monitoring
CpuLoadMeter cpuLoadMeter;
static uint32_t lastCpuLoadTime = 0;
static const uint32_t CPU_LOAD_INTERVAL_MS = 100; // Send CPU load every 100ms

// Bypass test: play the bound sample buffer directly (outside RNBO) to isolate click source
static volatile float* gBypassBuf = nullptr;     // planar float32 buffer (ownership retained by RNBO, read-only here)
static volatile size_t gBypassFramesPadded = 0;  // total frames including guard padding
static volatile size_t gBypassFramesOrig = 0;    // original frames without padding
static volatile int    gBypassChannels = 0;      // 1 or 2
static volatile int    gBypassSampleRate = 0;    // Hz of the sample data
static bool gBypassEnabled = false;
static double gBypassPhase = 0.0;                // fractional frame position for bypass playback
// Diagnostic playback mode: 0 = RNBO, 1 = Bypass Linear (default when bypass enabled),
// 2 = Bypass Nearest-Neighbor (no interpolation), 3 = 1kHz Sine,
// 4 = Constant Zero, 5 = Constant Sample (ch0[0])
static int gDiagMode = 1;
static float gSinePhase = 0.0f;
// Optional internal-SRAM scratch copy to test SDRAM bandwidth issues
// We'll mirror up to this many frames per channel from the start of the sample.
#ifndef BYPASS_SCRATCH_FRAMES
#define BYPASS_SCRATCH_FRAMES 24000 // ~0.5s @ 48kHz
#endif
static float gBypassScratchL[BYPASS_SCRATCH_FRAMES + 3] = {0};
static float gBypassScratchR[BYPASS_SCRATCH_FRAMES + 3] = {0};
static size_t gBypassScratchFrames = 0; // how many valid frames filled (<= BYPASS_SCRATCH_FRAMES)
// Index-stepping diagnostic table: seamless ramp [0..1) with guard padding
#ifndef INDEX_TEST_FRAMES
#define INDEX_TEST_FRAMES 4096
#endif
static float gIndexTest[INDEX_TEST_FRAMES + 3] = {0};
static inline void InitIndexTest()
{
	// Build a single-cycle sine table for a continuous periodic test
	for (size_t i = 0; i < INDEX_TEST_FRAMES; ++i)
	{
		float phase = (2.0f * 3.14159265359f) * ((float)i / (float)INDEX_TEST_FRAMES);
		gIndexTest[i] = sinf(phase);
	}
	// Guard copies of last sample (not used for wrap but keeps safe reads)
	const float last = gIndexTest[INDEX_TEST_FRAMES - 1];
	gIndexTest[INDEX_TEST_FRAMES + 0] = last;
	gIndexTest[INDEX_TEST_FRAMES + 1] = last;
	gIndexTest[INDEX_TEST_FRAMES + 2] = last;
}

// Simple SD browser state for choosing test files (moved to file scope)
static char   gAudioFiles[32][128]; // up to 32 entries, names relative to SD root
static int    gAudioFileCount = 0;
static int    gCurrentFileIndex = -1;
static inline bool HasExt(const char* name, const char* ext1, const char* ext2 = nullptr)
{
	// Case-insensitive check whether name ends with ext1 or ext2
	auto endsWithCI = [](const char* s, const char* suf) -> bool {
		size_t ns = strlen(s), nf = strlen(suf);
		if (ns < nf) return false;
		for (size_t i = 0; i < nf; ++i) {
			char a = s[ns - nf + i];
			char b = suf[i];
			if (a >= 'A' && a <= 'Z') a = (char)(a - 'A' + 'a');
			if (b >= 'A' && b <= 'Z') b = (char)(b - 'A' + 'a');
			if (a != b) return false;
		}
		return true;
	};
	if (endsWithCI(name, ext1)) return true;
	if (ext2 && endsWithCI(name, ext2)) return true;
	return false;
}
static void ScanAudioFilesSD()
{
	gAudioFileCount = 0; gCurrentFileIndex = -1;
	DIR dir; FILINFO fno;
	// Note: libDaisy's FatFS build often disables long filenames (LFN),
	// so only use the short 8.3 name in fno.fname.
	if (f_opendir(&dir, "/") != FR_OK) return;
	// First pass: collect WAV first for preference
	while (gAudioFileCount < 32) {
		FRESULT fr = f_readdir(&dir, &fno);
		if (fr != FR_OK || fno.fname[0] == 0) break;
		if (fno.fname[0] == '.') continue;
		if (fno.fattrib & AM_DIR) continue;
		if (HasExt(fno.fname, ".wav", ".WAV")) {
			snprintf(gAudioFiles[gAudioFileCount], sizeof(gAudioFiles[0]), "%s", fno.fname);
			gAudioFileCount++;
		}
	}
	f_closedir(&dir);
	// Second pass: add AIFF if space remains
	if (gAudioFileCount < 32) {
		if (f_opendir(&dir, "/") != FR_OK) return;
		while (gAudioFileCount < 32) {
			FRESULT fr = f_readdir(&dir, &fno);
			if (fr != FR_OK || fno.fname[0] == 0) break;
			if (fno.fname[0] == '.') continue;
			if (fno.fattrib & AM_DIR) continue;
			if (HasExt(fno.fname, ".aif", ".aiff")) {
				snprintf(gAudioFiles[gAudioFileCount], sizeof(gAudioFiles[0]), "%s", fno.fname);
				gAudioFileCount++;
			}
		}
		f_closedir(&dir);
	}
	if (gAudioFileCount > 0) gCurrentFileIndex = 0;
}

// just an axample of how to control a named parameter
long Param1Index = -1;			  // this we will find out later
const char *Param1Name = "level"; // just an arbitrary name for the sake of this demo

// Function to send CPU load as MIDI CC 16
void SendCpuLoadAsMidi()
{
    // Get the current CPU load and scale to 0-100
    float cpuLoad = cpuLoadMeter.GetAvgCpuLoad();
    uint8_t ccValue = (uint8_t)(cpuLoad * 100.0f);
    
    // Clamp the value to 0-100 range
    if (ccValue > 100) ccValue = 100;
    
    // Send MIDI CC 16 (0xB0 + channel 0, CC 16, value)
    uint8_t midiData[3] = {0xB0, 16, ccValue};
    midi.SendMessage(midiData, 3);
}

// Typical Switch case for Message Type.
void HandleMidiMessage(MidiEvent m)
{
	if (m.type < MidiMessageType::SystemCommon)
	{
		uint8_t midiData[3];
		switch (m.type)
		{
		case NoteOff:
			midiData[0] = 0x80 + m.channel;
			break;
		case NoteOn:
			midiData[0] = 0x90 + m.channel;
			break;
		case PolyphonicKeyPressure:
			midiData[0] = 0xA0 + m.channel;
			break;
		case ControlChange:
			midiData[0] = 0xB0 + m.channel;
			break;
		case ProgramChange:
			midiData[0] = 0xC0 + m.channel;
			break;
		case ChannelPressure:
			midiData[0] = 0xD0 + m.channel;
			break;
		case PitchBend:
			midiData[0] = 0xE0 + m.channel;
			break;
		default:
			break;
		}

		midiData[1] = m.data[0];
		midiData[2] = m.data[1];

		rnbo.processMidiEvent(RNBO::RNBOTimeNow, 0, midiData, 3);
	}
}

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
	// Start CPU load measurement
	cpuLoadMeter.OnBlockStart();

	if (gBypassEnabled && gDiagMode == 3)
	{
		// 1kHz sine test tone to validate output path
		const float sr = hw.AudioSampleRate();
		const float w = 2.0f * 3.14159265359f * 1000.0f / sr;
		for (size_t n = 0; n < size; ++n)
		{
			float s = sinf(gSinePhase);
			gSinePhase += w;
			if (gSinePhase > 2.0f * 3.14159265359f)
				gSinePhase -= 2.0f * 3.14159265359f;
			out[0][n] = s;
			out[1][n] = s;
		}
	}
	else if (gBypassEnabled && gDiagMode == 4)
	{
		// Constant zero to test DAC/codec path
		for (size_t n = 0; n < size; ++n)
		{
			out[0][n] = 0.0f;
			out[1][n] = 0.0f;
		}
	}
	else if (gBypassEnabled && gDiagMode == 5)
	{
		// Constant sample (first sample of ch0, or 0 if not available)
		float c = 0.0f;
		if (gBypassBuf) {
			const float* buf = (const float*)gBypassBuf;
			c = buf[0];
		}
		for (size_t n = 0; n < size; ++n)
		{
			out[0][n] = c;
			out[1][n] = c;
		}
	}
	else if (gBypassEnabled && gDiagMode == 6)
	{
		// Index-stepping diagnostic using an internal periodic sine table
		const float hwSr = hw.AudioSampleRate();
		const float inc = 1.0f; // 1:1 stepping at hw rate (will wrap)
		for (size_t n = 0; n < size; ++n)
		{
			size_t i0 = (size_t)gBypassPhase;
			if (i0 >= INDEX_TEST_FRAMES) i0 -= INDEX_TEST_FRAMES;
			const float s0 = gIndexTest[i0];
			const float s1 = gIndexTest[(i0 + 1) % INDEX_TEST_FRAMES];
			const float frac = (float)(gBypassPhase - (double)i0);
			const float v = s0 + (s1 - s0) * frac;
			out[0][n] = v * 0.5f;
			out[1][n] = v * 0.5f;
			gBypassPhase += inc;
			if (gBypassPhase >= (double)INDEX_TEST_FRAMES)
				gBypassPhase -= (double)INDEX_TEST_FRAMES;
		}
	}
	else if (gBypassEnabled && gBypassBuf && gBypassFramesOrig > 1 && gBypassChannels > 0 && gBypassSampleRate > 0)
	{
		// Simple resampling from sample SR -> hardware SR using linear interpolation
		// Snapshot state at block start to avoid race with control thread updates.
		const float* buf = (const float*)gBypassBuf; // local snapshot (planar)
		// Always loop over the full original sample length; do NOT shrink to scratch window.
		const size_t framesOrig = gBypassFramesOrig;
		const int chs = gBypassChannels;
		const float hwSr = hw.AudioSampleRate();
		const float inc = (float)gBypassSampleRate / hwSr; // how many sample frames to advance per hw frame

		// Select source pointers: only use scratch if it fully covers the sample
		const bool useScratch = (gBypassScratchFrames > 0 && gBypassScratchFrames >= framesOrig);
		const size_t stride = (gBypassFramesPadded ? gBypassFramesPadded : gBypassFramesOrig);
		const float* ch0 = useScratch ? gBypassScratchL : (buf + 0 * stride);
		const float* ch1 = nullptr;
		if (chs > 1)
			ch1 = useScratch ? gBypassScratchR : (buf + 1 * stride);

		if (gDiagMode == 1)
		{
			// Linear interpolation
			for (size_t n = 0; n < size; ++n)
			{
				// Left
				size_t i0 = (size_t)gBypassPhase;
				float frac = (float)(gBypassPhase - (double)i0);
				// Clamp base index to valid range at true loop length (framesOrig)
				if (i0 >= framesOrig) {
					// wrap if something went off due to parameter change
					gBypassPhase -= framesOrig;
					i0 = (size_t)gBypassPhase;
				}
				// Neighbor sample with wrap-around to avoid an extra discontinuity at the loop point
				const size_t i1 = (i0 + 1 == framesOrig) ? 0 : (i0 + 1);

				const float s0L = ch0[i0];
				const float s1L = ch0[i1];
				float l = s0L + (s1L - s0L) * frac;

				float r = l;
				if (chs > 1)
				{
					const float s0R = ch1[i0];
					const float s1R = ch1[i1];
					r = s0R + (s1R - s0R) * frac;
				}

				out[0][n] = l;
				out[1][n] = r;

				gBypassPhase += inc;
				if (gBypassPhase >= (double)framesOrig)
					gBypassPhase -= (double)framesOrig;
			}
		}
	else if (gDiagMode == 2)
		{
			// Nearest-neighbor (no interpolation)
			for (size_t n = 0; n < size; ++n)
			{
		size_t i0 = (size_t)(gBypassPhase); // floor to avoid rounding jitter
				if (i0 >= framesOrig)
					i0 -= framesOrig;
				float l = ch0[i0];
				float r = l;
				if (chs > 1)
				{
					r = ch1[i0];
				}
				out[0][n] = l;
				out[1][n] = r;
				gBypassPhase += inc;
				if (gBypassPhase >= (double)framesOrig)
					gBypassPhase -= (double)framesOrig;
			}
		}
	}
	else
	{
		// Keep audio callback lean: process RNBO
		rnbo.process(in, 2, out, 2, size);
	}

	// End CPU load measurement
	cpuLoadMeter.OnBlockEnd();
}

// ---- SD audio file loader for RNBO buffer~ ----
// Supports: WAV (PCM 16-bit LE) and AIFF (PCM 16-bit BE). Returns interleaved float32 [-1,1].
// Includes an asynchronous state machine so we don't block MIDI/audio for large files.
namespace {
	struct LoadedAudio {
		float*  data = nullptr;   // interleaved: [f0c0, f0c1, f1c0, f1c1, ...]
		size_t  frames = 0;
		int     channels = 0;
		int     sampleRate = 0;
	};

	// Helpers for big/little endian decoding
	static inline uint16_t ReadLE16(const uint8_t* p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
	static inline uint32_t ReadLE32(const uint8_t* p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
	static inline uint16_t ReadBE16(const uint8_t* p) { return ((uint16_t)p[0] << 8) | (uint16_t)p[1]; }
	static inline uint32_t ReadBE32(const uint8_t* p) { return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3]; }

	// Convert 80-bit extended (AIFF) to double (approx).
	static double ReadExtended80(const uint8_t* p)
	{
		// 1 bit sign, 15-bit exponent (bias 16383), then 1 integer bit + 63 fraction bits
		uint16_t se = ((uint16_t)p[0] << 8) | p[1];
		int sign = (se & 0x8000) ? -1 : 1;
		int exp  = (se & 0x7FFF);
		uint64_t mant64 = ((uint64_t)p[2] << 56) | ((uint64_t)p[3] << 48) | ((uint64_t)p[4] << 40) | ((uint64_t)p[5] << 32)
						| ((uint64_t)p[6] << 24) | ((uint64_t)p[7] << 16) | ((uint64_t)p[8] << 8)  | (uint64_t)p[9];
		if (exp == 0 && mant64 == 0) return 0.0; // zero
		// mant64 includes the implicit integer bit as MSB; normalize to [1.0, 2.0)
		long double mant = (long double)mant64 / (long double)(1ULL << 63);
		int e = exp - 16383; // unbiased exponent
		long double val = (long double)sign * ldexp((double)mant, e);
		return (double)val;
	}

	static bool LoadWavFatFs(const char* path, LoadedAudio& out)
	{
		FIL f; FRESULT fr = f_open(&f, path, FA_READ);
		if (fr != FR_OK) return false;

		uint8_t hdr[12]; UINT br = 0; fr = f_read(&f, hdr, sizeof(hdr), &br); if (fr != FR_OK || br != sizeof(hdr)) { f_close(&f); return false; }
		if (memcmp(hdr, "RIFF", 4) != 0 || memcmp(hdr + 8, "WAVE", 4) != 0) { f_close(&f); return false; }

		// Parse chunks
		uint16_t fmtAudioFormat = 0; uint16_t fmtNumCh = 0; uint32_t fmtSR = 0; uint16_t fmtBits = 0;
		uint32_t dataPos = 0; uint32_t dataSize = 0;

		for (;;) {
			uint8_t chdr[8]; br = 0; fr = f_read(&f, chdr, 8, &br); if (fr != FR_OK || br != 8) break;
			uint32_t chunkSize = ReadLE32(chdr + 4);
			if (memcmp(chdr, "fmt ", 4) == 0) {
				uint8_t fmt[32] = {0}; UINT toread = chunkSize > sizeof(fmt) ? sizeof(fmt) : chunkSize;
				fr = f_read(&f, fmt, toread, &br); if (fr != FR_OK || br != toread) { f_close(&f); return false; }
				fmtAudioFormat = ReadLE16(fmt + 0);
				fmtNumCh = ReadLE16(fmt + 2);
				fmtSR = ReadLE32(fmt + 4);
				fmtBits = ReadLE16(fmt + 14);
				// Skip any remaining bytes in fmt chunk
				if (chunkSize > toread) f_lseek(&f, f_tell(&f) + (chunkSize - toread));
			} else if (memcmp(chdr, "data", 4) == 0) {
				dataPos = f_tell(&f);
				dataSize = chunkSize;
				// Move file pointer past data for now; we'll seek back to dataPos later
				f_lseek(&f, f_tell(&f) + chunkSize);
			} else {
				// Skip unknown chunk
				f_lseek(&f, f_tell(&f) + chunkSize);
			}
			// Chunks are padded to even size
			if (chunkSize & 1) f_lseek(&f, f_tell(&f) + 1);
			// Stop if we got both fmt and data
			if (fmtNumCh && dataPos) break;
		}

		if (!(fmtAudioFormat == 1 /*PCM*/ && fmtBits == 16 && fmtNumCh >= 1 && dataPos > 0)) { f_close(&f); return false; }

		size_t totalSamples = dataSize / 2; // 16-bit
	size_t frames = totalSamples / fmtNumCh;
	// Allocate interleaved buffer [frame][ch]
	float* buf = (float*)RNBO::Platform::malloc(frames * fmtNumCh * sizeof(float));
		if (!buf) { f_close(&f); return false; }

		// Read and convert in chunks
		f_lseek(&f, dataPos);
		const size_t chunkSamps = 4096 * fmtNumCh;
		int16_t* tmp = (int16_t*)RNBO::Platform::malloc(chunkSamps * sizeof(int16_t));
		if (!tmp) { RNBO::Platform::free(buf); f_close(&f); return false; }
		size_t samplesRemaining = totalSamples; size_t writeOffset = 0;
		while (samplesRemaining > 0) {
			size_t thisSamps = samplesRemaining > chunkSamps ? chunkSamps : samplesRemaining;
			UINT bytesToRead = (UINT)(thisSamps * sizeof(int16_t));
			br = 0; fr = f_read(&f, tmp, bytesToRead, &br);
			if (fr != FR_OK || br != bytesToRead) { RNBO::Platform::free(tmp); RNBO::Platform::free(buf); f_close(&f); return false; }
			for (size_t i = 0; i < thisSamps; ++i) buf[writeOffset + i] = (float)tmp[i] / 32768.0f;
			writeOffset += thisSamps; samplesRemaining -= thisSamps;
		}
		RNBO::Platform::free(tmp);
		f_close(&f);

		out.data = buf; out.frames = frames; out.channels = fmtNumCh; out.sampleRate = (int)fmtSR;
		return true;
	}

	static bool LoadAiffFatFs(const char* path, LoadedAudio& out)
	{
		FIL f; FRESULT fr = f_open(&f, path, FA_READ);
		if (fr != FR_OK) return false;
		uint8_t hdr[12]; UINT br = 0; fr = f_read(&f, hdr, sizeof(hdr), &br); if (fr != FR_OK || br != sizeof(hdr)) { f_close(&f); return false; }
		if (memcmp(hdr, "FORM", 4) != 0 || memcmp(hdr + 8, "AIFF", 4) != 0) { f_close(&f); return false; }

		int channels = 0; uint32_t frames = 0; int bits = 0; int sampleRate = 44100;
		uint32_t ssndDataPos = 0; uint32_t ssndDataSize = 0; uint32_t ssndOffset = 0;

		// Iterate chunks
		while (true) {
			uint8_t chdr[8]; br = 0; fr = f_read(&f, chdr, 8, &br); if (fr != FR_OK || br != 8) break;
			uint32_t chunkSize = ReadBE32(chdr + 4);
			if (memcmp(chdr, "COMM", 4) == 0) {
				uint8_t comm[18]; if (chunkSize < 18) { f_close(&f); return false; }
				fr = f_read(&f, comm, 18, &br); if (fr != FR_OK || br != 18) { f_close(&f); return false; }
				channels = ReadBE16(comm + 0);
				frames = ReadBE32(comm + 2);
				bits = ReadBE16(comm + 6);
				sampleRate = (int)ReadExtended80(comm + 8);
				// Skip any remaining
				if (chunkSize > 18) f_lseek(&f, f_tell(&f) + (chunkSize - 18));
			} else if (memcmp(chdr, "SSND", 4) == 0) {
				uint8_t ss[8]; if (chunkSize < 8) { f_close(&f); return false; }
				fr = f_read(&f, ss, 8, &br); if (fr != FR_OK || br != 8) { f_close(&f); return false; }
				ssndOffset = ReadBE32(ss + 0);
				// uint32_t blockSize = ReadBE32(ss + 4);
				ssndDataPos = (uint32_t)f_tell(&f) + ssndOffset;
				ssndDataSize = chunkSize - 8 - ssndOffset;
				// Move to end of this chunk
				f_lseek(&f, f_tell(&f) + (chunkSize - 8));
			} else {
				// Skip chunk
				f_lseek(&f, f_tell(&f) + chunkSize);
			}
			if (chunkSize & 1) f_lseek(&f, f_tell(&f) + 1);
		}

		if (!(channels >= 1 && bits == 16 && ssndDataPos > 0 && frames > 0)) { f_close(&f); return false; }
		// Bound data size to frames * channels * 2
		uint64_t expectedBytes = (uint64_t)frames * (uint64_t)channels * 2ULL;
		if (ssndDataSize < expectedBytes) frames = (uint32_t)(ssndDataSize / (channels * 2));

	// Allocate interleaved buffer
	float* buf = (float*)RNBO::Platform::malloc((size_t)frames * (size_t)channels * sizeof(float));
		if (!buf) { f_close(&f); return false; }

		// Read and convert big-endian 16-bit
		f_lseek(&f, ssndDataPos);
		const size_t chunkSamps = 4096 * channels; // samples = frames*channels
		uint8_t* tmp = (uint8_t*)RNBO::Platform::malloc(chunkSamps * 2);
		if (!tmp) { RNBO::Platform::free(buf); f_close(&f); return false; }
		size_t samplesRemaining = (size_t)frames * (size_t)channels; size_t writeOffset = 0;
		while (samplesRemaining > 0) {
			size_t thisSamps = samplesRemaining > chunkSamps ? chunkSamps : samplesRemaining;
			UINT bytesToRead = (UINT)(thisSamps * 2);
			br = 0; fr = f_read(&f, tmp, bytesToRead, &br);
			if (fr != FR_OK || br != bytesToRead) { RNBO::Platform::free(tmp); RNBO::Platform::free(buf); f_close(&f); return false; }
			for (size_t i = 0; i < thisSamps; ++i) {
				int16_t s = (int16_t)ReadBE16(tmp + (i * 2));
				buf[writeOffset + i] = (float)s / 32768.0f;
			}
			writeOffset += thisSamps; samplesRemaining -= thisSamps;
		}
		RNBO::Platform::free(tmp);
		f_close(&f);

		out.data = buf; out.frames = frames; out.channels = channels; out.sampleRate = sampleRate;
		return true;
	}

	static void BindLoadedAudioToRNBO(const LoadedAudio& au)
	{
		if (!au.data || au.frames == 0 || au.channels <= 0) return;
		// RNBO's Float32Buffer expects channel-contiguous (planar) layout.
		// Our loaders produce interleaved data. Convert to planar if needed.
		float* dataToBind = au.data;
		size_t framesOut = au.frames; // final frame count we will advertise to RNBO
		bool havePlanar = (au.channels == 1); // mono interleaved == planar
		if (au.channels > 1)
		{
			const size_t total = au.frames * (size_t)au.channels;
			float* planar = (float*)RNBO::Platform::malloc(total * sizeof(float));
			if (planar)
			{
				for (int ch = 0; ch < au.channels; ++ch)
				{
					float* dst = planar + ((size_t)ch * au.frames);
					const float* src = au.data + ch; // interleaved source starting at channel offset
					for (size_t i = 0; i < au.frames; ++i)
					{
						dst[i] = src[i * (size_t)au.channels];
					}
				}
				// Free interleaved buffer after converting
				RNBO::Platform::free(au.data);
				dataToBind = planar;
				havePlanar = true;
			}
		}

		// Add a small per-channel guard pad to avoid boundary interpolation clicks
		if (dataToBind)
		{
			const size_t framesPadded = au.frames + 3; // guard of 3 samples per channel
			const size_t totalPadded = framesPadded * (size_t)au.channels;
			float* padded = (float*)RNBO::Platform::malloc(totalPadded * sizeof(float));
			if (padded)
			{
				if (!havePlanar)
				{
					// Convert mono interleaved -> planar (trivial copy) while padding
					const float* src = dataToBind; // mono contiguous
					float* dst = padded; // single channel
					for (size_t i = 0; i < au.frames; ++i) dst[i] = src[i];
					float last = au.frames ? src[au.frames - 1] : 0.f;
					dst[au.frames + 0] = last;
					dst[au.frames + 1] = last;
					dst[au.frames + 2] = last;
				}
				else
				{
					// Already planar: copy each channel segment and pad
					for (int ch = 0; ch < au.channels; ++ch)
					{
						const float* src = dataToBind + (size_t)ch * au.frames;
						float* dst = padded + (size_t)ch * framesPadded;
						for (size_t i = 0; i < au.frames; ++i) dst[i] = src[i];
						float last = au.frames ? src[au.frames - 1] : 0.f;
						dst[au.frames + 0] = last;
						dst[au.frames + 1] = last;
						dst[au.frames + 2] = last;
					}
				}
				// Replace buffer with padded planar
				if (dataToBind != au.data)
					RNBO::Platform::free(dataToBind);
				else
					RNBO::Platform::free(au.data);
				dataToBind = padded;
				framesOut = framesPadded; // advertise padded length only if padding succeeded
			}
		}

		// Get DataRef for sample1 (index 1 per exported patch)
		RNBO::DataRef* ref = rnbo.getDataRef(1);
		if (!ref) return;
		// Describe float32 audio buffer (planar channel layout)
		RNBO::DataType dt;
		dt.type = RNBO::DataType::Float32AudioBuffer;
		dt.audioBufferInfo.channels = (size_t)au.channels;
		dt.audioBufferInfo.samplerate = (RNBO::number)au.sampleRate;
		ref->setType(dt);
		// Provide planar data: [ch0 .. frames][ch1 .. frames] ...
		// Data size reflects whether padding succeeded (framesOut updated above)
		size_t bytes = framesOut * (size_t)au.channels * sizeof(float);
		ref->setData(reinterpret_cast<char*>(dataToBind), bytes, true);
		rnbo.processDataViewUpdate(1, RNBO::RNBOTimeNow);

		// Expose to bypass tester (read-only). RNBO owns memory; we only read it.
		gBypassBuf = dataToBind;
		gBypassFramesPadded = framesOut;
		gBypassFramesOrig = au.frames;
		gBypassChannels = au.channels;
		gBypassSampleRate = au.sampleRate;

		// Fill internal scratch from start of sample to avoid SDRAM fetches in bypass
		gBypassScratchFrames = 0;
		if (au.frames > 0)
		{
			const size_t copyFrames = au.frames < (size_t)BYPASS_SCRATCH_FRAMES ? au.frames : (size_t)BYPASS_SCRATCH_FRAMES;
			const size_t stride = gBypassFramesPadded ? gBypassFramesPadded : au.frames;
			// Channel 0
			for (size_t i = 0; i < copyFrames; ++i) gBypassScratchL[i] = dataToBind[0 * stride + i];
			float lastL = copyFrames ? gBypassScratchL[copyFrames - 1] : 0.f;
			gBypassScratchL[copyFrames + 0] = lastL;
			gBypassScratchL[copyFrames + 1] = lastL;
			gBypassScratchL[copyFrames + 2] = lastL;
			// Channel 1 (if present)
			if (au.channels > 1)
			{
				for (size_t i = 0; i < copyFrames; ++i) gBypassScratchR[i] = dataToBind[1 * stride + i];
				float lastR = copyFrames ? gBypassScratchR[copyFrames - 1] : 0.f;
				gBypassScratchR[copyFrames + 0] = lastR;
				gBypassScratchR[copyFrames + 1] = lastR;
				gBypassScratchR[copyFrames + 2] = lastR;
			}
			gBypassScratchFrames = copyFrames;
		}
	}

	// Blocking load helper: load full file before audio starts
	static bool BlockingLoadSample(const char* filename)
	{
		LoadedAudio au{};
		// Build full path using FatFS root
		char full[256];
		const char* root = fsi.GetSDPath();
		if (root && root[0])
			snprintf(full, sizeof(full), "%s%s", root, filename[0] == '/' ? filename + 1 : filename);
		else
			snprintf(full, sizeof(full), "%s", filename);

		// Prefer based on extension, but try both formats
		bool tried = false;
		const char* dot = strrchr(filename, '.');
		if (dot)
		{
			char ext[8];
			snprintf(ext, sizeof(ext), "%s", dot);
			for (char* p = ext; *p; ++p) *p = (char)tolower(*p);
			if (!strcmp(ext, ".aif") || !strcmp(ext, ".aiff"))
			{
				tried = true;
				if (LoadAiffFatFs(full, au)) { BindLoadedAudioToRNBO(au); return true; }
				if (LoadWavFatFs(full, au))  { BindLoadedAudioToRNBO(au); return true; }
			}
			else if (!strcmp(ext, ".wav"))
			{
				tried = true;
				if (LoadWavFatFs(full, au))  { BindLoadedAudioToRNBO(au); return true; }
				if (LoadAiffFatFs(full, au)) { BindLoadedAudioToRNBO(au); return true; }
			}
		}
		// Unknown or no extension: try AIFF then WAV
		if (!tried)
		{
			if (LoadAiffFatFs(full, au)) { BindLoadedAudioToRNBO(au); return true; }
			if (LoadWavFatFs(full, au))  { BindLoadedAudioToRNBO(au); return true; }
		}
		return false;
	}

	// Non-blocking loader
	struct AsyncLoader {
	enum Type { NONE, WAV16, AIFF16 } type = NONE;
		FIL f{}; bool open = false; bool done = false; bool failed = false;
		uint32_t dataPos = 0; uint32_t dataBytes = 0; // WAV
		uint32_t aiffPos = 0; uint32_t aiffBytes = 0; // AIFF
	int channels = 0; int samplerate = 0; size_t frames = 0;
	float* buf = nullptr; size_t writeSamples = 0; // interleaved samples written
	uint8_t tmp[8192]; // temp bytes read from disk

		void Reset() {
			if (open) { f_close(&f); open = false; }
			type = NONE; done = false; failed = false;
			dataPos = dataBytes = aiffPos = aiffBytes = 0;
			channels = samplerate = 0; frames = 0; writeSamples = 0;
			if (buf) { RNBO::Platform::free(buf); buf = nullptr; }
		}

		bool Begin(const char* filename) {
			Reset();
			// Build full path from FatFS root
			char full[256]; const char* root = fsi.GetSDPath();
			if (root && root[0]) snprintf(full, sizeof(full), "%s%s", root, filename[0] == '/' ? filename + 1 : filename);
			else snprintf(full, sizeof(full), "%s", filename);
			if (f_open(&f, full, FA_READ) != FR_OK) { failed = true; return false; }
			open = true;
			// Peek header to choose type
			uint8_t hdr[12]; UINT br = 0; if (f_read(&f, hdr, sizeof(hdr), &br) != FR_OK || br != sizeof(hdr)) { failed = true; return false; }
			if (!memcmp(hdr, "RIFF", 4) && !memcmp(hdr + 8, "WAVE", 4)) {
				// WAV: scan chunks for fmt/data
				uint16_t fmtAudio = 0, fmtBits = 0; uint16_t ch = 0; uint32_t sr = 0; uint32_t dpos = 0, dbytes = 0;
				for (;;) {
					uint8_t chdr[8]; br = 0; if (f_read(&f, chdr, 8, &br) != FR_OK || br != 8) break;
					uint32_t csize = ReadLE32(chdr + 4);
					if (!memcmp(chdr, "fmt ", 4)) {
						uint8_t fmt[32] = {0}; UINT toread = csize > sizeof(fmt) ? sizeof(fmt) : csize;
						if (f_read(&f, fmt, toread, &br) != FR_OK || br != toread) { failed = true; return false; }
						fmtAudio = ReadLE16(fmt + 0); ch = ReadLE16(fmt + 2); sr = ReadLE32(fmt + 4); fmtBits = ReadLE16(fmt + 14);
						if (csize > toread) f_lseek(&f, f_tell(&f) + (csize - toread));
					} else if (!memcmp(chdr, "data", 4)) {
						dpos = f_tell(&f); dbytes = csize; f_lseek(&f, f_tell(&f) + csize);
					} else { f_lseek(&f, f_tell(&f) + csize); }
					if (csize & 1) f_lseek(&f, f_tell(&f) + 1);
					if (ch && dpos) break;
				}
				if (!(fmtAudio == 1 && fmtBits == 16 && ch >= 1 && dpos > 0)) { failed = true; return false; }
				channels = ch; samplerate = (int)sr; frames = (size_t)(dbytes / 2) / (size_t)ch;
				// Allocate interleaved
				buf = (float*)RNBO::Platform::malloc(frames * (size_t)ch * sizeof(float)); if (!buf) { failed = true; return false; }
				dataPos = dpos; dataBytes = dbytes; f_lseek(&f, dataPos); type = WAV16; return true;
			}
			if (!memcmp(hdr, "FORM", 4)) {
				// Check form type
				if (memcmp(hdr + 8, "AIFF", 4) != 0) { failed = true; return false; }
				// Parse aiff COMM + SSND
				int ch = 0, bits = 0; uint32_t fr = 0; int sr = 44100; uint32_t ssPos = 0, ssBytes = 0; uint32_t ssOffset = 0;
				while (true) {
					uint8_t chdr[8]; br = 0; if (f_read(&f, chdr, 8, &br) != FR_OK || br != 8) break;
					uint32_t csize = ReadBE32(chdr + 4);
					if (!memcmp(chdr, "COMM", 4)) {
						if (csize < 18) { failed = true; return false; }
						uint8_t comm[18]; if (f_read(&f, comm, 18, &br) != FR_OK || br != 18) { failed = true; return false; }
						ch = ReadBE16(comm + 0); fr = ReadBE32(comm + 2); bits = ReadBE16(comm + 6); sr = (int)ReadExtended80(comm + 8);
						if (csize > 18) f_lseek(&f, f_tell(&f) + (csize - 18));
					} else if (!memcmp(chdr, "SSND", 4)) {
						if (csize < 8) { failed = true; return false; }
						uint8_t ss[8]; if (f_read(&f, ss, 8, &br) != FR_OK || br != 8) { failed = true; return false; }
						ssOffset = ReadBE32(ss + 0); ssPos = (uint32_t)f_tell(&f) + ssOffset; ssBytes = csize - 8 - ssOffset;
						f_lseek(&f, f_tell(&f) + (csize - 8));
					} else { f_lseek(&f, f_tell(&f) + csize); }
					if (csize & 1) f_lseek(&f, f_tell(&f) + 1);
				}
				if (!(ch >= 1 && bits == 16 && ssPos > 0 && fr > 0)) { failed = true; return false; }
				channels = ch; samplerate = sr; frames = ssBytes / 2 / (size_t)ch;
				// Allocate interleaved
				buf = (float*)RNBO::Platform::malloc(frames * (size_t)ch * sizeof(float));
				if (!buf) { failed = true; return false; }
				aiffPos = ssPos; aiffBytes = ssBytes; f_lseek(&f, aiffPos); type = AIFF16; return true;
			}
			failed = true; return false;
		}

		// Call repeatedly from main loop. Returns true while still working.
	bool Step(size_t maxBytesPerStep = 4096) {
			if (!open || done || failed || type == NONE) return false;
			UINT br = 0; size_t samplesToDo = 0;
			if (type == WAV16) {
				if (dataBytes == 0) { Finish(); return false; }
				size_t bytes = dataBytes > maxBytesPerStep ? maxBytesPerStep : dataBytes;
				if (bytes & 1) bytes--; // even
				if (bytes == 0) { Finish(); return false; }
				if (f_read(&f, tmp, (UINT)bytes, &br) != FR_OK || br != bytes) { failed = true; return false; }
				samplesToDo = bytes / 2; // 16-bit
				for (size_t i = 0; i < samplesToDo; ++i) {
					int16_t s = (int16_t)ReadLE16(tmp + i * 2);
					buf[writeSamples + i] = (float)s / 32768.0f;
				}
				writeSamples += samplesToDo; dataBytes -= bytes;
				if (dataBytes == 0) { Finish(); return false; }
				return true;
			} else if (type == AIFF16) {
				if (aiffBytes == 0) { Finish(); return false; }
		size_t bytes = aiffBytes > maxBytesPerStep ? maxBytesPerStep : aiffBytes;
		if (bytes & 1) bytes--;
		if (bytes == 0) { Finish(); return false; }
				if (f_read(&f, tmp, (UINT)bytes, &br) != FR_OK || br != bytes) { failed = true; return false; }
				samplesToDo = bytes / 2;
				for (size_t i = 0; i < samplesToDo; ++i) {
					int16_t s = (int16_t)ReadBE16(tmp + i * 2);
					buf[writeSamples + i] = (float)s / 32768.0f;
				}
				writeSamples += samplesToDo; aiffBytes -= bytes;
				if (aiffBytes == 0) { Finish(); return false; }
				return true;
			}
			return false;
		}

		void Finish() {
			if (open) { f_close(&f); open = false; }
			done = true;
		}
	};

	AsyncLoader g_loader;
}

int main(void)
{
	hw.Init();

	// Prepare index test diagnostic buffer
	InitIndexTest();

	// Ensure FPU does not stall on subnormal (denormal) floats.
	// This matches typical audio DSP setups (flush-to-zero + denormals-are-zero)
	// and helps avoid periodic clicks from denormal slow paths.
	#if defined(__FPU_PRESENT) && (__FPU_PRESENT == 1)
		// Set default FP state for exceptions: FPDSCR
		// Bit 24 = FZ (Flush-to-zero), Bit 25 = DN (Default NaN / denormals-are-zero behavior)
		FPU->FPDSCR |= (1u << 24) | (1u << 25);
		// Also set current FPSCR bits for this context
		uint32_t fpscr = __get_FPSCR();
		fpscr |= (1u << 24) | (1u << 25);
		__set_FPSCR(fpscr);
	#endif

	SdmmcHandler::Config sd_cfg;
    sd_cfg.Defaults();
    sdcard.Init(sd_cfg);
    fsi.Init(FatFSInterface::Config::MEDIA_SD);
    f_mount(&fsi.GetSDFileSystem(), "/", 1);

	hw.SetAudioBlockSize(vectorsize);
	hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);

	// Create the TLSF pool in SDRAM starting after a reserved low region.
	// https://electro-smith.github.io/libDaisy/md_doc_2md_2__a6___getting-_started-_external-_s_d_r_a_m.html
	myPool = tlsf_create_with_pool((void *)POOLBASE, POOLSIZE);

	// initialize the Daisy for receiving MIDI over USB
	MidiUsbHandler::Config midi_cfg;
	midi_cfg.transport_config.periph = MidiUsbTransport::Config::INTERNAL;
	midi.Init(midi_cfg);

	// initialize CPU load monitoring
	cpuLoadMeter.Init(hw.AudioSampleRate(), hw.AudioBlockSize());

	// initialize RNBO, here for example audio samples are allocated in the SDRAM (through our allocator)
	rnbo.initialize();

	// if you do not want this to allocate (and a slightly better performance) consider exporting your code
	// with a fixed audio vector size matching the one you are using (vectorsize)
	rnbo.prepareToProcess(48000, vectorsize, true);

	// find out the index of our named parameter to be able to interact with it later
	for (int i = 0; i < rnbo.getNumParameters(); i++)
	{
		auto name = rnbo.getParameterName(i);
		if (strcmp(name, Param1Name) == 0)
		{
			Param1Index = i;
		}
	}

	// Ensure output is audible by default if a 'level' parameter exists
	if (Param1Index >= 0)
	{
		rnbo.setParameterValueNormalized(Param1Index, 1.0f);
	}

	// Start audio immediately; load sample asynchronously in the main loop to avoid long startup silence
	hw.StartAdc();
	hw.StartAudio(AudioCallback);

	// Prefer loading user WAV/AIFF from SD root for apples-to-apples with WavPlayer.
	// If none found, fall back to RNBO DataRef default filename.
	ScanAudioFilesSD();
	if (gAudioFileCount > 0) {
		g_loader.Begin(gAudioFiles[gCurrentFileIndex]);
	} else {
		const RNBO::DataRef* dr = rnbo.getDataRef(1);
		const char* fname = dr && dr->getFile() && dr->getFile()[0] ? dr->getFile() : "impulse.aif";
		if (!g_loader.Begin(fname)) {
			const char* dot = strrchr(fname, '.');
			if (dot) {
				char alt[256]; strncpy(alt, fname, sizeof(alt)); alt[sizeof(alt)-1] = '\0';
				if ((dot[1] | 32) == 'a' && (dot[2] | 32) == 'i') strcpy(alt + (dot - fname), ".wav");
				else strcpy(alt + (dot - fname), ".aif");
				g_loader.Begin(alt);
			}
		}
	}

	for (;;)
	{
		// Poll MIDI and handle events outside the audio thread
		midi.Listen();
		while (midi.HasEvents())
		{
			HandleMidiMessage(midi.PopEvent());
		}

		// Process UI controls here (keeps callback lean)
		hw.ProcessAllControls();
		if (Param1Index >= 0)
		{
			auto inc = hw.encoder.Increment();
			if (inc != 0)
			{
				auto val = rnbo.getParameterValue(Param1Index);
				val += inc == 1 ? 0.1f : -0.1f;
				if (val > 1.0f) val = 1.0f; else if (val < 0.0f) val = 0.0f;
				rnbo.setParameterValueNormalized(Param1Index, val);
			}
		}


		// Toggle bypass on Button 2 press (edge-triggered)
		static bool prevBtn2 = false;
		bool btn2 = hw.button2.Pressed();
		if (btn2 && !prevBtn2)
		{
			gBypassEnabled = !gBypassEnabled;
			gBypassPhase = 0.0; // restart loop from start for determinism
		}
		prevBtn2 = btn2;



		// Button 1: when bypass is ON, cycle diagnostic modes; when bypass is OFF, cycle audio files
		static bool prevBtn1 = false;
		bool btn1 = hw.button1.Pressed();
		if (btn1 && !prevBtn1)
		{
			if (gBypassEnabled)
			{
				gDiagMode++;
				if (gDiagMode > 6) gDiagMode = 1;
				gBypassPhase = 0.0;
				gSinePhase = 0.0f;
			}
			else if (gAudioFileCount > 0)
			{
				gCurrentFileIndex = (gCurrentFileIndex + 1) % gAudioFileCount;
				g_loader.Reset();
				g_loader.Begin(gAudioFiles[gCurrentFileIndex]);
				gBypassPhase = 0.0;
			}
		}
		prevBtn1 = btn1;

	// LED feedback by mode:
	// RNBO (bypass off): Green
	// Bypass Linear: Blue
	// Bypass Nearest: Magenta
	// Sine: White
	// Zero: Cyan, Constant: Yellow, Index test: Red
		if (!gBypassEnabled)
		{
			hw.led1.Set(0.0f, 1.0f, 0.0f);
			hw.led2.Set(0.0f, 0.0f, 0.0f);
		}
		else if (gDiagMode == 1)
		{
			hw.led1.Set(0.0f, 0.0f, 1.0f);
			hw.led2.Set(0.0f, 0.0f, 0.0f);
		}
		else if (gDiagMode == 2)
		{
			hw.led1.Set(1.0f, 0.0f, 1.0f);
			hw.led2.Set(0.0f, 0.0f, 0.0f);
		}
	else if (gDiagMode == 3) { hw.led1.Set(1.0f, 1.0f, 1.0f); hw.led2.Set(0.0f, 0.0f, 0.0f); }
	else if (gDiagMode == 4) { hw.led1.Set(0.0f, 1.0f, 1.0f); hw.led2.Set(0.0f, 0.0f, 0.0f); }
	else if (gDiagMode == 5) { hw.led1.Set(1.0f, 1.0f, 0.0f); hw.led2.Set(0.0f, 0.0f, 0.0f); }
	else /* gDiagMode == 6 */ { hw.led1.Set(1.0f, 0.0f, 0.0f); hw.led2.Set(0.0f, 0.0f, 0.0f); }
		hw.UpdateLeds();

		// Advance async file loading without blocking (small chunks to minimize contention)
		static uint32_t lastLoadTick = 0;
		uint32_t tick = System::GetTick();
		if (!g_loader.done && !g_loader.failed && (tick - lastLoadTick) >= 1) {
			lastLoadTick = tick;
			bool more = g_loader.Step(2048);
			if (g_loader.done && !g_loader.failed && g_loader.buf) {
				LoadedAudio au; au.data = g_loader.buf; au.channels = g_loader.channels; au.frames = g_loader.frames; au.sampleRate = g_loader.samplerate;
				g_loader.buf = nullptr; // transfer ownership to RNBO
				BindLoadedAudioToRNBO(au);
			}
			if (more) System::Delay(0);
		}

		// Send CPU load periodically from the non-audio thread to avoid USB work in the audio callback
		uint32_t now = System::GetTick();
		if (now - lastCpuLoadTime >= CPU_LOAD_INTERVAL_MS)
		{
			if (!gBypassEnabled) SendCpuLoadAsMidi();
			lastCpuLoadTime = now;
		}
	}
}
