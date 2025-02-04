#include "Additive.h"
#include "Calib.h"
#include "cordic.h"
#include <cstring>

Additive additive;

// Create sine look up table as constexpr so will be stored in flash
//constexpr std::array<float, Additive::sinLUTSize> sineLUT = additive.CreateSinLUT();
float sineLUT[Additive::sinLUTSize] __attribute__((section (".dtcm")));

Additive::Additive()
{
	for (uint32_t s = 0; s < sinLUTSize; ++s){
		sineLUT[s] = std::sin(s * 2.0f * std::numbers::pi / sinLUTSize);
	}

	multipliers[0] = 0.3f;
	multipliers[9] = 0.0f;
	multipliers[99] = 0.3f;
}


void Additive::CalcSample()
{
	debugPin2.SetHigh();

	// Previously calculated samples output at beginning of interrupt to keep timing independent of calculation time
	SPI2->TXDR = (int32_t)(outputSamples[0] * scaleOutput);
	SPI2->TXDR = (int32_t)(outputSamples[1] * scaleOutput);

	// Pitch calculations
	if (octaveBtn.Pressed()) {
		cfg.octaveDown = !cfg.octaveDown;
		if (cfg.octaveDown) {
			octaveLED.SetHigh();
		} else {
			octaveLED.SetLow();
		}
	}
	const float octave = cfg.octaveDown ? 2 : 1;
	const uint32_t inc = calib.pitchLUT[adc.Pitch_CV] / octave;		// Increment of the base sine read position
	uint32_t incMult = inc;											// Increment of the harmonic sine read position

	// Calculate the maximum harmonic before aliasing (experimenting shows we need to truncate slightly before the nyquist frequency)
	aliasHarmonic = std::min(maxHarmonic, (uint32_t)(((float)sinLUTSize / 2.25f) / (inc >> sinLUTShift)));

	float mixOut[2] = {0.0f, 0.0f};

	for (uint32_t i = 0; i < aliasHarmonic; ++i) {
		readPos[i] += incMult;
		mixOut[i & 1] += sineLUT[readPos[i] >> sinLUTShift] * multipliers[i];
		incMult += inc;
	}

	outputSamples[0] = FastTanh(mixOut[0]);
	outputSamples[1] = FastTanh(mixOut[1]);

	debugPin2.SetLow();
}


inline void Additive::Smooth(float& currentValue, const float newValue, const float fraction)
{
	currentValue = currentValue * fraction + (1.0f - fraction) * newValue;
}


float Additive::FastTanh(const float x)
{
	// Apply FastTan approximation to limit sample from -1 to +1
	float x2 = x * x;
	float a = x * (135135.0f + x2 * (17325.0f + x2 * (378.0f + x2)));
	float b = 135135.0f + x2 * (62370.0f + x2 * (3150.0f + x2 * 28.0f));
	return a / b;
}


void Additive::IdleJobs()
{
	debugPin1.SetHigh();

	const bool lpf = filterMode.IsLow();

	static constexpr float smoothOld = 0.95f;
	static constexpr float smoothNew = 1.0f - smoothOld;
	static constexpr float startMultLPF = smoothNew * (200.0f / 65535.0f);
	static constexpr float startMultComb = smoothNew * (50.0f / 65535.0f);		// Start becomes interval between comb frequencies

	if (lpf) {
		// LPF
		static constexpr float slopeMult = (1.0f / 65535.0f);
		float slope = adc.Filter_Slope_Pot * slopeMult;		// FIXME - slope also has CV input
		filterSlope = filterSlope * smoothOld + (1.0f - (slope * slope)) * smoothNew;		// Square the slope to give better control at low settings

	} else {
		// Comb filter
		static constexpr float slopeMult = smoothNew * (0.1f / 65535.0f);
		filterSlope = filterSlope * smoothOld + (adc.Filter_Slope_Pot + 5000) * slopeMult;		// FIXME - slope also has CV input
	}

	const float cvA = std::max(61300.0f - adc.Filter_A_CV, 0.0f);		// Reduce to ensure can hit zero with noise
	const float cvB = std::max(61300.0f - adc.Filter_B_CV, 0.0f);		// Reduce to ensure can hit zero with noise
	filterStart[0] = filterStart[0] * smoothOld + std::clamp((adc.Filter_A_Pot + cvA), 0.0f, 65535.0f) * (lpf ? startMultLPF : startMultComb);
	filterStart[1] = filterStart[1] * smoothOld + std::clamp((adc.Filter_B_Pot + cvB), 0.0f, 65535.0f) * (lpf ? startMultLPF : startMultComb);

	// Set filter LED brightness - FIXME - very inefficient
	filterLED_A = (filterStart[0] * 4096) / (lpf ? 200 : 50);
	filterLED_B = (filterStart[1] * 4096) / (lpf ? 200 : 50);

	uint32_t combPos[2] = {0, 0};
	int32_t combDir[2] = {1, 1};


	if (harmonicMode.IsLow()) {				// Mode to scale multipliers according to a cosine wave shape

		const float maxLevel = 0.15f;
		float sineScale[2] = {maxLevel, maxLevel};
		uint16_t sinePos = Additive::sinLUTSize / 4;		// Start at maximum (pi/2)

		// Calculate smoothed spread amount from pot and CV with trimmer controlling range of CV
		static constexpr float spreadMult = 10000.0f / (float)sinLUTSize;
		const float cv = std::max(61300.0f - adc.Harm_Stretch_CV, 0.0f);		// Reduce to ensure can hit zero with noise

		Smooth(multSpread, 100.0f + (adc.Harm_Stretch_Pot + (NormaliseADC(adc.Harm_Stretch_Trm) * cv)) * spreadMult, 0.99f);
		float spread = multSpread;

		static constexpr float growMult = 0.05f * (30.0f / (float)sinLUTSize);
		multGrow = multGrow * 0.95f + (adc.Harm_Warp_CV - 32768) * growMult;

		for (uint32_t i = 0; i < maxHarmonics; ++i) {
			FilterCalc(i, sineScale[i & 1], combPos[i & 1], combDir[i & 1], maxLevel);

			// Reduce the level of the last two harmonics to avoid glitching
			if (i == maxHarmonics - 2) sineScale[i & 1] *= 0.5f;
			if (i == maxHarmonics - 1) sineScale[i & 1] *= 0.25f;

			multipliers[i] = sineScale[i & 1] * (1.0f + sineLUT[sinePos & sinLUTBits]);
			if (spread + multGrow > 50.0f) {			// Check the sine wave position will increment enough
				spread += multGrow;
			}
			sinePos += spread;
		}

	} else {								// Mode to spread individual harmonics

		const float maxLevel = 0.3f;
		float startLevel[2] = {maxLevel, maxLevel};

		// Calculate smoothed spread amount from pot and CV with trimmer controlling range of CV
		static constexpr float spreadMult = 10.0f / 65535.0f;
		const float cv = std::max(61300.0f - adc.Harm_Stretch_CV, 0.0f);		// Reduce to ensure can hit zero with noise
		multSpread = (0.99f * multSpread) +
				(0.01f * (1.0f + (adc.Harm_Stretch_Pot + NormaliseADC(adc.Harm_Stretch_Trm) * cv) * spreadMult));

		static constexpr float growMult = 0.05f * (1.0f / 65535.0f);
		multGrow = multGrow * 0.95f + (adc.Harm_Warp_CV - 32768) * growMult;
		float spread = multSpread;

		float spreadHarm = 1.0f + spread;

		multipliers[0] = startLevel[0];
		multipliers[1] = startLevel[1];

		float nextVal = 0.0f;		// For storing partial value when spread creates fractional harmonic

		for (uint32_t i = 2; i < maxHarmonics; ++i) {
			FilterCalc(i, startLevel[i & 1], combPos[i & 1], combDir[i & 1], maxLevel);

			// Reduce the level of the last two harmonics to avoid glitching
			if (i == maxHarmonics - 2) startLevel[i & 1] *= 0.5f;
			if (i == maxHarmonics - 1) startLevel[i & 1] *= 0.25f;

			// Increase the spread of harmonics dividing fractional components between the current and next multiplier
			uint32_t intPart = (uint32_t)spreadHarm;
			if (intPart == i) {
				float fractPart = spreadHarm - intPart;
				multipliers[i] = (nextVal + 1.0f - fractPart) * startLevel[i & 1];
				nextVal = fractPart;
				spreadHarm += spread;
				if (spread + multGrow > 3.0f) {
					spread += multGrow;
				}

			} else if (intPart > i && nextVal > 0.0f) {
				// if increasing the spread skips the next harmonic then apply the residual fraction to the next harmonic here
				multipliers[i] = nextVal * startLevel[i & 1];
				nextVal = 0.0f;

			} else {
				multipliers[i] = 0.0f;
			}
		}
	}

	debugPin1.SetLow();
}


inline void Additive::FilterCalc(uint32_t pos, float& scale, uint32_t& combPos, int32_t& combDir, float maxLevel)
{
	bool lpf = filterMode.IsLow();
	bool notch = false;

	if (lpf) {						// Exponential LP filter
		if (pos >= filterStart[pos & 1]) {
			if (scale > 0.001f) {
				scale *= filterSlope;
			} else {
				scale = 0.0f;
			}
		}
	} else {
		if (notch) {
			// Repeating Notch filter - filterStart sets harmonics between combs; filterSlope sets slope of combs
			++combPos;
			if (combPos > std::round(filterStart[pos & 1])) {
				if (combDir > 0) {				// falling
					scale -= filterSlope;
					if (scale <= 0) {
						scale = 0;
						combDir *= -1;
					}
				} else {						// rising
					scale += filterSlope;
					if (scale >= maxLevel) {
						scale = maxLevel;
						combDir *= -1;
						combPos = 0;
					}
				}
			}
		} else {
			// Comb filter
			if (combDir > 0) {				// Start falling
				scale -= filterSlope;
				if (scale <= 0) {
					scale = 0;
					combDir = 0;
				}
			} else if (combDir == 0) {		// Then wait until next spike
				if (++combPos > std::round(filterStart[pos & 1])) {
					combDir = -1;
				}
			} else {						// Then rising
				scale += filterSlope;
				if (scale >= maxLevel) {
					scale = maxLevel;
					combDir = 1;
					combPos = 0;
				}

			}

		}
	}
}


void Additive::UpdateConfig()
{

}
