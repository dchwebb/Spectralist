#pragma once

#include <string_view>

#include "initialisation.h"
#include "Filter.h"
#include "FatTools.h"
#include "configManager.h"
#include "UI.h"
#include "Additive.h"

struct WaveTable {
	friend class CDCHandler;					// Allow the serial handler access to private data for debug printing
	friend class Config;						// Allow the config access to private data to save settings
	friend class UI;
	friend class Additive;
public:
	void CalcSample();							// Called by interrupt handler to generate next sample
	void Init();								// Initialise caches, buffers etc
	void CalcAdditive();
	bool LoadWaveTable(uint32_t* startAddr);
	void Draw();
	void UpdateWavetableList();
	void ChangeWaveTable(const int32_t index);
	void ChannelBOctave(bool change = false);	// Called when channel B octave button is pressed
	void WarpButton(bool change);				// Called when warp button is pressed
	float QuantisedWavetablePos(const uint8_t chn);	// For drawing: return quantised wavetable position
	static void UpdateConfig();
	void FixUnaligned();

	struct {
		char wavetable[8];
		bool octaveChnB = false;
		bool warpButton = false;
		uint32_t additiveWaves = 0x12346789;
	} cfg;

	ConfigSaver configSaver = {
		.settingsAddress = &cfg,
		.settingsSize = sizeof(cfg),
		.validateSettings = &WaveTable::UpdateConfig
	};

	enum class Warp {none, squeeze, bend, mirror, tzfm, count} warpType = Warp::none;		//, reverse
	static constexpr std::string_view warpNames[] = {"No Warp", "Squeeze", "Bend", "Mirror", "TZFM"};		// "Reverse",
	static constexpr uint32_t harmonicCount = 10;

	enum class AdditiveType : uint8_t {none = 0, sine1 = 1, sine2 = 2, sine3 = 3, sine4 = 4, sine5 = 5, sine6 = 6, square = 7, saw = 8, triangle = 9};
	uint32_t harmonicSets;
	float additiveHarmonics[8][harmonicCount];

	static constexpr uint32_t sinLUTSize = 2048;
	constexpr auto CreateSinLUT()									// constexpr function to generate LUT in Flash
	{
		std::array<float, sinLUTSize + 1> array {};					// Create one extra entry to simplify interpolation
		for (uint32_t s = 0; s < sinLUTSize + 1; ++s){
			array[s] = std::sin(s * 2.0f * std::numbers::pi / sinLUTSize);
		}
		return array;
	}

private:
	static constexpr uint32_t maxWavetable = 1000;
	static constexpr size_t lfnSize = 12;							// Widest string that can be displayed
	static constexpr float scaleOutput = -std::pow(2.0f, 31.0f);	// Multiple to convert -1.0 - 1.0 float to 32 bit int and invert
	static constexpr float scaleVCAOutput = scaleOutput / 65536.0f;	// To scale when VCA is used
	enum Invalid : uint8_t {OK = 0, Fragmented, HeaderCorrupt, SampleFormat, ChannelCount, EmptyFolder, Unaligned, End};
	const char* InvalidText[Invalid::End] {"OK", "Fragged", "Corrupt", "Format", "Channels", "Empty", "Unaligned"};

	struct Wav {
		char name[8];
		char lfn[lfnSize];
		uint32_t dir;						// Index into the wavList array to entry containing directory item
		uint32_t size;						// Size of file in bytes
		uint32_t cluster;					// Starting cluster
		uint32_t lastCluster;				// If file spans multiple clusters store last cluster before jump - if 0xFFFFFFFF then clusters are contiguous
		const uint8_t* startAddr;			// Address of data section
		union {
			uint32_t dataSize;				// Size of data section in bytes
			uint32_t firstWav;				// For directories holds the index of the first file
		};
		uint32_t sampleCount;				// Number of samples (stereo samples only counted once)
		uint8_t byteDepth;					// 4 = 32 bit, 2 = 16 bit etc
		uint16_t dataFormat;				// 1 = PCM; 3 = Float
		uint8_t channels;					// 1 = mono, 2 = stereo
		uint16_t tableCount;				// Number of 2048 sample wavetables in file
		uint32_t metadata;					// Serum metadata
		SampleType sampleType;
		Invalid invalid;					// Code indicating why wav is invalid
		bool isDir;
		bool fragmented;
	} wavList[maxWavetable];

	// In case file is fragmented store cluster chain on loading
	struct clusterList {
		const uint8_t* startAddr;			// Address of data section
		const uint8_t* endAddr;				// End Address of data section
	} fragChain[16];


	void OutputSample(const uint8_t channel, const float ratio);
	float FastTanh(const float x);
	float CalcWarp();
	void AdditiveWave();
	void GetWavInfo(Wav& wav);
	void ReadDir(const FATFileInfo* dirEntry, const uint32_t dirIndex);
	void CleanLFN(char* storeName);
	static bool WavetableSorter(Wav const& lhs, Wav const& rhs);
	void FragChain();

	float defaultWavetable[3 * 2048];			// Built-in wavetables
	float outputSamples[2] = {0.0f, 0.0f};		// Preprepared samples sent to DAC on interrupt
	float oldOutputSamples[2] = {0.0f, 0.0f};	// Previous output samples used for cross-fading
	float crossfade = 0.0f;						// Amount of cross-fade

	uint32_t activeWaveTable;					// Index of active wavetable in wavList
	uint32_t wavetableCount;					// number of wavetables and directories found in file system

	float smoothedInc = 0.0f;					// For smoothing pitch CV
	float pitchInc[2] = {0.0f, 0.0f};			// Pitch increment - reciprocal used in anti-aliasing filter calculations
	float readPos[2] = {0.0f, 0.0f};			// Wavetable read position for each channel

	bool stepped = false;						// Store Stepped/Smooth switch position
	int32_t warpTypeVal = 0;					// Used for setting hysteresis on warp type
	float warpAmt = 0.0f;						// Used for smoothing control values

	char longFileName[100];						// Holds long file name as it is made from multiple fat entries
	uint8_t lfnPosition = 0;

	uint8_t drawData[2][UI::waveDrawWidth];		// Holds scaled output samples from both channels for use in wavetable display
	static constexpr float drawWidthMult = (float)(UI::waveDrawWidth - 1.0f) / 2048.0f;		// Scale to width of the LCD draw area
	static constexpr float drawHeightMult = (float)(UI::waveDrawHeight - 4) / 2.0f;		// Scale to height of the LCD draw area

	struct {
		volatile uint16_t& adcPot;
		volatile uint16_t& adcCV;
		volatile uint16_t* adcTrimmer;
		float pos;		// Smoothed ADC value

		float Val() {
			constexpr float scaleOutput = 0.01f / 65536.0f;			// scale constant for two 16 bit values and filter
			const float trimmer = (adcTrimmer == nullptr) ? 1.0f : NormaliseADC(*adcTrimmer);
			const float cv = std::max(61300.0f - adcCV, 0.0f);		// Reduce to ensure can hit zero with noise
			pos = (0.99f * pos) + std::clamp((adcPot + trimmer * cv), 0.0f, 65535.0f) * scaleOutput;
			return pos;
		}
	} wavetablePos[2] = {{adc.Wavetable_Pos_A_Pot, adc.WavetablePosA_CV, &adc.Wavetable_Pos_A_Trm, 0.0f},
						 {adc.Wavetable_Pos_B_Pot, adc.WavetablePosB_CV, nullptr, 0.0f}};


	static inline float NormaliseADC(uint16_t adcVal)
	{
		static constexpr float adcDivider = 1.0f / 55000.0f;		// reduce divider to allow maximum of slightly over 1.0f
		return adcDivider * adcVal;
	}

	GpioPin modeSwitch	{GPIOE, 2, GpioPin::Type::Input};			// PE2: Mode switch
	GpioPin octaveUp	{GPIOD, 0, GpioPin::Type::InputPulldown};	// PD0: Octave_Up
	GpioPin octaveDown	{GPIOD, 1, GpioPin::Type::InputPulldown};	// PD1: Octave_Down
	GpioPin chBMix		{GPIOD, 8, GpioPin::Type::InputPulldown};	// PD8: ChB_Mix
	GpioPin chBRingMod	{GPIOD, 9, GpioPin::Type::InputPulldown};	// PD9: ChB_RM

	GpioPin octaveLED	{GPIOD, 14, GpioPin::Type::Output};			// PD14: LED_Oct_B

};

extern WaveTable wavetable;
