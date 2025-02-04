#pragma once

#include "initialisation.h"
#include "configManager.h"

class Calib {
public:
	Calib();
	void Calibrate(char key = 0);
	static void UpdatePitchLUT();
	static void UpdateConfig();

	// 0v = 62110; 1v = 52810; 2v = 43520; 3v = 34220; 4v = 24920; 5v = 15620; 6V = 6320
	// C: 16.35 Hz 32.70 Hz; 65.41 Hz; 130.81 Hz; 261.63 Hz; 523.25 Hz; 1046.50 Hz; 2093.00 Hz; 4186.01 Hz
	// 62100 > 65.41 Hz; 52800 > 130.81 Hz; 43520 > 261.63 Hz

	// Increment = Hz * (65536 / 48000) = Hz * (sinLUTsize / samplerate)
	// Pitch calculations - Increase pitchBase to increase pitch; Reduce ABS(cvMult) to increase spread

	// Calculations: 9300 is difference in cv between two octaves; 52800 is cv at 1v and 130.81 is desired Hz at 1v

	static constexpr float pitchBaseDef = (65.41f * (65536.0f / sampleRate)) / std::pow(2.0, -52810.0f / 9300.0f);
	static constexpr float pitchMultDef = -1.0f / 9300.0f;

	struct {
		float pitchBase = pitchBaseDef;
		float pitchMult = pitchMultDef;
	} cfg;

	ConfigSaver configSaver = {
		.settingsAddress = &cfg,
		.settingsSize = sizeof(cfg),
		.validateSettings = UpdateConfig
	};

	uint32_t pitchLUT[adcMax + 1];

	bool calibrating;			// Triggered by serial console
	enum class State {Waiting0, Waiting1, Octave0, Octave1, PendingSave};
	State state;

	float adcOctave0;
	float adcOctave1;
	uint32_t calibCount = 0;
};

extern Calib calib;
