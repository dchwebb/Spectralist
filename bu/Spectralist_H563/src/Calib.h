#pragma once

#include "initialisation.h"
#include "configManager.h"


class Calib {
public:
	Calib();
	void Calibrate(char key = 0);
	static void UpdatePitchLUT();

	// 0v = 61200; 1v = 50110; 2v = 39020; 3v = 27910; 4v = 16790; 5v = 5670
	// C: 16.35 Hz 32.70 Hz; 65.41 Hz; 130.81 Hz; 261.63 Hz; 523.25 Hz; 1046.50 Hz; 2093.00 Hz; 4186.01 Hz
	// 61200 > 65.41 Hz; 50110 > 130.81 Hz; 39020 > 261.63 Hz

	// Increment = Hz * (2048 / 48000) = Hz * (wavetablesize / samplerate)
	// Pitch calculations - Increase pitchBase to increase pitch; Reduce ABS(cvMult) to increase spread

	// Calculations: 11330 is difference in cv between two octaves; 50050 is cv at 1v and 130.81 is desired Hz at 1v

	struct {
		float pitchBase = (65.41f * (2048.0f / sampleRate)) / std::pow(2.0, -50050.0f / 11330.0f);
		float pitchMult = -1.0f / 11330.0f;
	} cfg;

	ConfigSaver configSaver = {
		.settingsAddress = &cfg,
		.settingsSize = sizeof(cfg),
		.validateSettings = UpdatePitchLUT
	};

	float pitchLUT[adcMax + 1];

	bool calibrating;			// Triggered by serial console

private:


	enum class State {Waiting0, Waiting1, Octave0, Octave1, PendingSave};
	State state;

	float adcOctave0;
	float adcOctave1;
	uint32_t calibCount = 0;
};

extern Calib calib;
