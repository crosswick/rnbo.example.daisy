#include "daisy_pod.h"
#include "util/CpuLoadMeter.h"
#include "sys/system.h"
#include "tlsf.h"
#include "ff.h" // FatFS

#define RNBO_USE_FLOAT32
#define RNBO_NOTHROW
#define RNBO_FIXEDLISTSIZE 64
#define RNBO_USECUSTOMALLOCATOR

#include "export/rnbo_source.h"

tlsf_t myPool;
#define SDRAM_BASE  (0xC0000000u)
// Use the entire external SDRAM (64 MB) for the TLSF pool
#define POOLBASE    (SDRAM_BASE)
#define POOLSIZE    (64u * 1024u * 1024u)

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

DaisyPod hw;
MidiUsbHandler midi;

CpuLoadMeter cpuMeter;

// Small helpers for sending MIDI CC debug values
static inline uint8_t ClampCC(int v) { return (uint8_t)(v < 0 ? 0 : (v > 127 ? 127 : v)); }
static inline uint8_t Map01ToCC(float x)
{
	if (!(x == x)) return 0; // NaN guard
	int v = (int)(x * 127.0f + 0.5f);
	return ClampCC(v);
}
static inline void SendCC(uint8_t cc, uint8_t val, uint8_t channel = 0)
{
	uint8_t msg[3] = {(uint8_t)(0xB0 | channel), cc, val};
	midi.SendMessage(msg, 3);
}

SdmmcHandler   sdcard;
FatFSInterface fsi;

RNBO::rnbomatic<> rnbo;
static constexpr int kDataRefIndex = 1;
static constexpr uint32_t kSampleRateHz = 48000;
static constexpr size_t kBlockSize = 128;
static constexpr size_t kMaxNameLen = 128;


void HandleMidiMessage(const MidiEvent& m)
{
	if (m.type < MidiMessageType::SystemCommon)
	{
		uint8_t midiData[3] = {0, 0, 0};
		switch (m.type)
		{
			case NoteOff:                midiData[0] = 0x80 + m.channel; break;
			case NoteOn:                 midiData[0] = 0x90 + m.channel; break;
			case PolyphonicKeyPressure:  midiData[0] = 0xA0 + m.channel; break;
			case ControlChange:          midiData[0] = 0xB0 + m.channel; break;
			case ProgramChange:          midiData[0] = 0xC0 + m.channel; break;
			case ChannelPressure:        midiData[0] = 0xD0 + m.channel; break;
			case PitchBend:              midiData[0] = 0xE0 + m.channel; break;
			default: break;
		}
		midiData[1] = m.data[0];
		midiData[2] = m.data[1];
		rnbo.processMidiEvent(RNBO::RNBOTimeNow, 0, midiData, 3);
	}
}

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
	// Keep audio callback lean: measure CPU and process RNBO only
	cpuMeter.OnBlockStart();
	rnbo.process(in, 2, out, 2, size);
	cpuMeter.OnBlockEnd();
}

// SD WAV (PCM16) async loader: decodes to planar float32 for RNBO buffer~
namespace {
	// Little-endian helpers
	static inline uint16_t ReadLE16(const uint8_t* p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
	static inline uint32_t ReadLE32(const uint8_t* p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }

	// Non-blocking loader (WAV16 only)
	struct AsyncLoader {
		FIL f{}; bool open = false; bool done = false; bool failed = false;
		uint32_t dataBytes = 0; // WAV bytes remaining
		uint32_t totalBytes = 0; // total WAV payload bytes (for progress)
		int channels = 0; int sampleRate = 0; size_t frames = 0;
		float* buf = nullptr; size_t writeFrames = 0; // planar frames written
		uint8_t tmp[2048];

		void Reset() {
			if (open) { f_close(&f); open = false; }
			done = false; failed = false;
			dataBytes = 0; totalBytes = 0;
			channels = sampleRate = 0; frames = 0; writeFrames = 0;
			if (buf) { RNBO::Platform::free(buf); buf = nullptr; }
		}

		bool Begin(const char* filename) {
			Reset();
			// Build full path from SD root
			char full[256]; const char* root = fsi.GetSDPath();
			if (root && root[0]) snprintf(full, sizeof(full), "%s%s", root, filename[0] == '/' ? filename + 1 : filename);
			else snprintf(full, sizeof(full), "%s", filename);
			if (f_open(&f, full, FA_READ) != FR_OK) { failed = true; return false; }
			open = true;
			// Peek header
			uint8_t hdr[12]; UINT br = 0; if (f_read(&f, hdr, sizeof(hdr), &br) != FR_OK || br != sizeof(hdr)) { failed = true; return false; }
			if (!memcmp(hdr, "RIFF", 4) && !memcmp(hdr + 8, "WAVE", 4)) {
				// Scan chunks for fmt/data
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
				channels = ch; sampleRate = (int)sr; frames = (size_t)(dbytes / 2) / (size_t)ch;
				// Allocate planar: [ch0..frames][ch1..frames] ...
				buf = (float*)RNBO::Platform::malloc(frames * (size_t)ch * sizeof(float)); if (!buf) { failed = true; return false; }
				dataBytes = dbytes; totalBytes = dbytes; f_lseek(&f, dpos); return true;
			}
			failed = true; return false;
		}

		// Call repeatedly from main loop; returns true while working
		bool Step(size_t maxBytesPerStep = 2048) {
			if (!open || done || failed) return false;
			if (dataBytes == 0) { Finish(); return false; }
			UINT br = 0;
			size_t bytes = dataBytes > maxBytesPerStep ? maxBytesPerStep : dataBytes;
			if (bytes > sizeof(tmp)) bytes = sizeof(tmp);
			// Align to whole frames (channels * 2 bytes)
			const size_t frameBytes = (size_t)channels * 2u;
			bytes -= (bytes % frameBytes);
			if (bytes == 0) { Finish(); return false; }
			if (f_read(&f, tmp, (UINT)bytes, &br) != FR_OK || br != bytes) { failed = true; return false; }
			const size_t nframes = bytes / frameBytes;
			// Deinterleave into planar buffer
			for (size_t fidx = 0; fidx < nframes; ++fidx) {
				const size_t base = fidx * (size_t)channels;
				for (int ch = 0; ch < channels; ++ch) {
					const size_t si = (base + (size_t)ch) * 2u;
					int16_t s = (int16_t)ReadLE16(tmp + si);
					buf[(size_t)ch * frames + (writeFrames + fidx)] = (float)s / 32768.0f;
				}
			}
			writeFrames += nframes; dataBytes -= bytes;
			if (dataBytes == 0) { Finish(); return false; }
			return true;
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

	SdmmcHandler::Config sd_cfg;
    sd_cfg.Defaults();
    sdcard.Init(sd_cfg);
    fsi.Init(FatFSInterface::Config::MEDIA_SD);
    f_mount(&fsi.GetSDFileSystem(), "/", 1);

	hw.SetAudioBlockSize(kBlockSize);
	hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);

	// Create the TLSF pool in SDRAM (entire region).
	myPool = tlsf_create_with_pool((void *)POOLBASE, POOLSIZE);

	// initialize the Daisy for receiving MIDI over USB
	MidiUsbHandler::Config midi_cfg;
	midi_cfg.transport_config.periph = MidiUsbTransport::Config::INTERNAL;
	midi.Init(midi_cfg);


	// initialize RNBO, here for example audio samples are allocated in the SDRAM (through our allocator)
	rnbo.initialize();

	// if you do not want this to allocate (and a slightly better performance) consider exporting your code
	// with a fixed audio vector size matching the one you are using
	rnbo.prepareToProcess(kSampleRateHz, kBlockSize, true);

	// CPU load meter for telemetry
	cpuMeter.Init((float)kSampleRateHz, (int)kBlockSize, 1.0f);


	// Start audio first so MIDI works immediately
	hw.StartAudio(AudioCallback);

	// Resolve RNBO buffer~ file (basename only) and begin async loading
	const RNBO::DataRef* dr = rnbo.getDataRef(kDataRefIndex);
	const char* requested = dr && dr->getFile() ? dr->getFile() : nullptr;
	if (requested && requested[0]) {
		// Extract basename using strrchr
		const char* slash = strrchr(requested, '/');
		const char* bslash = strrchr(requested, '\\');
		const char* base = slash && bslash ? (slash > bslash ? slash + 1 : bslash + 1)
		                                   : (slash ? slash + 1 : (bslash ? bslash + 1 : requested));
		if (base && *base) {
			char name[kMaxNameLen];
			snprintf(name, sizeof(name), "%s", base);
			g_loader.Begin(name);
		}
	}

	// MIDI CC telemetry timers
	constexpr uint8_t kCpuCcNumber  = 16;
	constexpr uint8_t kProgCcNumber = 17; // loader progress
	constexpr uint8_t kBeatCcNumber = 20; // heartbeat
	constexpr uint8_t kMidiChannel  = 0;  // Channel 1
	uint32_t          nextCpuMs     = System::GetNow() + 100;
	uint32_t          nextBeatMs    = System::GetNow() + 500;
	uint8_t           beatVal       = 0; // toggles 0/127

	for (;;)
	{
		// Poll MIDI and handle events outside the audio thread
		midi.Listen();
		while (midi.HasEvents())
		{
			HandleMidiMessage(midi.PopEvent());
		}


		// Advance async file loading without blocking
		if (!g_loader.done && !g_loader.failed) {
			g_loader.Step();
			if (g_loader.done && !g_loader.failed && g_loader.buf) {
				// Bind planar float32 audio directly to RNBO DataRef 1
				RNBO::DataRef* ref = rnbo.getDataRef(kDataRefIndex);
				if (ref) {
					RNBO::DataType dt;
					dt.type = RNBO::DataType::Float32AudioBuffer;
					dt.audioBufferInfo.channels = (size_t)g_loader.channels;
					dt.audioBufferInfo.samplerate = (RNBO::number)g_loader.sampleRate;
					ref->setType(dt);
					const size_t bytes = g_loader.frames * (size_t)g_loader.channels * sizeof(float);
					ref->setData(reinterpret_cast<char*>(g_loader.buf), bytes, true);
					rnbo.processDataViewUpdate(kDataRefIndex, RNBO::RNBOTimeNow);
					g_loader.buf = nullptr; // ownership transferred
				}
			}
		}

		// Send CPU load as CC16 at ~10Hz
		const uint32_t now = System::GetNow();
		if ((int32_t)(now - nextCpuMs) >= 0)
		{
			const float avg = cpuMeter.GetAvgCpuLoad(); // 0..1 or NaN before first block
			if (avg == avg) // check for NaN
			{
				SendCC(kCpuCcNumber, Map01ToCC(avg), kMidiChannel);
			}
			nextCpuMs += 100; // schedule next
		}

		// Send loader progress on CC17 while loading (0..127)
		if (!g_loader.done && !g_loader.failed && g_loader.totalBytes > 0)
		{
			float p = 1.0f - (float)g_loader.dataBytes / (float)g_loader.totalBytes;
			SendCC(kProgCcNumber, Map01ToCC(p), kMidiChannel);
		}

		// 2Hz heartbeat on CC20 (0/127)
		if ((int32_t)(now - nextBeatMs) >= 0)
		{
			beatVal = beatVal ? 0 : 127;
			SendCC(kBeatCcNumber, beatVal, kMidiChannel);
			nextBeatMs += 500;
		}

	}
}
