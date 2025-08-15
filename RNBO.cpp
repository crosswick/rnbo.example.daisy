#include "daisy_pod.h"
#include "tlsf.h"

#define RNBO_USE_FLOAT32
#define RNBO_NOTHROW
#define RNBO_FIXEDLISTSIZE 64
#define RNBO_USECUSTOMALLOCATOR

#include "export/rnbo_source.h"

// implement our custom allocation methods, which are just re-directing all calls to
// a simple pool based allocator

tlsf_t myPool;

// use all the available 64 MB in the SDRAM for our pool
#define POOLSIZE (64 * 1024 * 1024)

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

static const size_t vectorsize = 64;

DaisyPod hw;
MidiUsbHandler midi;

RNBO::rnbomatic<> rnbo;

// CPU load monitoring
CpuLoadMeter cpuLoadMeter;
static uint32_t lastCpuLoadTime = 0;
static const uint32_t CPU_LOAD_INTERVAL_MS = 100; // Send CPU load every 100ms

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
	
	hw.ProcessAllControls();
	while (midi.HasEvents())
	{
		HandleMidiMessage(midi.PopEvent());
	}

	if (Param1Index >= 0)
	{
		auto inc = hw.encoder.Increment();
		if (inc != 0)
		{
			auto val = rnbo.getParameterValue(Param1Index);
			val += inc == 1 ? 0.1 : -0.1;
			if (val > 1)
				val = 1;
			else if (val < 0)
				val = 0;
			rnbo.setParameterValueNormalized(Param1Index, val);
		}
	}

	rnbo.process(in, 2, out, 2, vectorsize);
	
	// End CPU load measurement
	cpuLoadMeter.OnBlockEnd();
	
	// Check if it's time to send CPU load (every 100ms)
	uint32_t currentTime = System::GetTick();
	if (currentTime - lastCpuLoadTime >= CPU_LOAD_INTERVAL_MS)
	{
		SendCpuLoadAsMidi();
		lastCpuLoadTime = currentTime;
	}
}

int main(void)
{
	hw.Init();
	hw.SetAudioBlockSize(vectorsize);
	hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);

	// directly use the base address of the SDRAM as a pool, see:
	// https://electro-smith.github.io/libDaisy/md_doc_2md_2__a6___getting-_started-_external-_s_d_r_a_m.html
	myPool = tlsf_create_with_pool((void *)0xC0000000, POOLSIZE);

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

	hw.StartAdc();
	hw.StartAudio(AudioCallback);
	for (;;)
	{
		midi.Listen();
	}
}
