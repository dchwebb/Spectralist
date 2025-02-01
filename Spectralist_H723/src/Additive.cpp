#include "Additive.h"
#include "Calib.h"
#include <cstring>

Additive additive;

// Create sine look up table as constexpr so will be stored in flash
constexpr std::array<float, Additive::sinLUTSize> sineLUT = additive.CreateSinLUT();
//float sineLUT[Additive::sinLUTSize];

Additive::Additive()
{
	//CreateSinLUT(sineLUT);
	multipliers[0] = 0.3f;
	multipliers[9] = 0.0f;
	multipliers[99] = 0.3f;
	//readPos = {0.0f, 8192.0f, 0.0f, 12288.0f, 0.0f, 40960.0f, 0.0f, 26542.0f, 21954.0f, 18022.0f, 9011.0f, 39649.0f, 22773.0f, 45711.0f, 17694.0f, 35880.0f, 58654.0f, 43417.0f, 27688.0f, 52756.0f, 5079.0f, 20643.0f, 33259.0f, 16056.0f, 39157.0f, 53903.0f, 12124.0f, 63242.0f, 14745.0f, 36536.0f, 4259.0f, 54722.0f, 11304.0f, 45547.0f, 17530.0f, 9666.0f, 2457.0f, 57507.0f, 43581.0f, 32604.0f, 15073.0f, 64389.0f, 46858.0f, 21790.0f, 52264.0f, 10158.0f, 24903.0f, 19169.0f, 52101.0f, 61603.0f, 20643.0f, 27852.0f, 5570.0f, 44728.0f, 6717.0f, 43253.0f, 327.0f, 13434.0f, 5734.0f, 20316.0f, 17039.0f, 10158.0f, 22118.0f, 33914.0f, 42270.0f, 16384.0f, 2621.0f, 62586.0f, 29491.0f, 6553.0f, 47349.0f, 327.0f, 8192.0f, 38830.0f, 53084.0f, 23265.0f, 26050.0f, 38010.0f, 36044.0f, 53903.0f, 50626.0f, 40468.0f, 7045.0f, 46202.0f, 64225.0f, 10321.0f, 7536.0f, 20316.0f, 59965.0f, 47349.0f, 33914.0f, 327.0f, 4751.0f, 30801.0f, 10649.0f, 39321.0f, 46694.0f, 23756.0f, 35880.0f, 52920.0f, 13926.0f, 13926.0f, 32276.0f, 58327.0f, 51281.0f, 46202.0f, 48005.0f, 53575.0f, 52428.0f, 14581.0f, 38174.0f, 38338.0f, 34078.0f, 45219.0f, 42598.0f, 2785.0f, 50790.0f, 13434.0f, 46202.0f, 34897.0f, 9830.0f, 44564.0f, 31457.0f, 10485.0f, 28344.0f, 29491.0f, 38338.0f, 57671.0f, 29163.0f, 49971.0f, 50790.0f, 15892.0f, 28672.0f, 19005.0f, 58654.0f, 60456.0f, 21463.0f, 42926.0f, 1146.0f, 61767.0f, 5406.0f, 11796.0f, 59637.0f, 21135.0f, 29163.0f, 12451.0f, 64225.0f, 48168.0f, 34406.0f, 46530.0f, 59310.0f, 61931.0f, 63569.0f, 6225.0f, 59310.0f, 55541.0f, 7864.0f, 24576.0f, 46530.0f, 59146.0f, 3112.0f, 19660.0f, 42434.0f, 35553.0f, 11304.0f, 4915.0f, 7864.0f, 58327.0f, 53411.0f, 47185.0f, 25067.0f, 23265.0f, 20152.0f, 64880.0f, 36864.0f, 55869.0f, 25559.0f, 15073.0f, 29982.0f, 36372.0f, 27033.0f, 41451.0f, 39976.0f, 49643.0f, 57344.0f, 163.0f, 2621.0f, 52592.0f, 41615.0f, 20480.0f, 64061.0f, 64716.0f, 49479.0f, 58163.0f, 55050.0f, 63897.0f, 5570.0f, 47513.0f, 15892.0f, 40796.0f};
	//readPos = {0, 8192, 0, 12288, 0, 40960, 0, 26542, 21954, 18022, 9011, 39649, 22773, 45711, 17694, 35880, 58654, 43417, 27688, 52756, 5079, 20643, 33259, 16056, 39157, 53903, 12124, 63242, 14745, 36536, 4259, 54722, 11304, 45547, 17530, 9666, 2457, 57507, 43581, 32604, 15073, 64389, 46858, 21790, 52264, 10158, 24903, 19169, 52101, 61603, 20643, 27852, 5570, 44728, 6717, 43253, 327, 13434, 5734, 20316, 17039, 10158, 22118, 33914, 42270, 16384, 2621, 62586, 29491, 6553, 47349, 327, 8192, 38830, 53084, 23265, 26050, 38010, 36044, 53903, 50626, 40468, 7045, 46202, 64225, 10321, 7536, 20316, 59965, 47349, 33914, 327, 4751, 30801, 10649, 39321, 46694, 23756, 35880, 52920, 13926, 13926, 32276, 58327, 51281, 46202, 48005, 53575, 52428, 14581, 38174, 38338, 34078, 45219, 42598, 2785, 50790, 13434, 46202, 34897, 9830, 44564, 31457, 10485, 28344, 29491, 38338, 57671, 29163, 49971, 50790, 15892, 28672, 19005, 58654, 60456, 21463, 42926, 1146, 61767, 5406, 11796, 59637, 21135, 29163, 12451, 64225, 48168, 34406, 46530, 59310, 61931, 63569, 6225, 59310, 55541, 7864, 24576, 46530, 59146, 3112, 19660, 42434, 35553, 11304, 4915, 7864, 58327, 53411, 47185, 25067, 23265, 20152, 64880, 36864, 55869, 25559, 15073, 29982, 36372, 27033, 41451, 39976, 49643, 57344, 163, 2621, 52592, 41615, 20480, 64061, 64716, 49479, 58163, 55050, 63897, 5570, 47513, 15892, 40796};
}

void Additive::CalcSample()
{
	debugPin2.SetHigh();

	// Previously calculated samples output at beginning of interrupt to keep timing independent of calculation time
	SPI2->TXDR = (int32_t)(outputSamples[0] * scaleOutput);
	SPI2->TXDR = (int32_t)(outputSamples[1] * scaleOutput);

	// Pitch calculations
	//const float octave = octaveDown.IsHigh() ? 0.5 : 1.0f;		// FIXME - octave is a momentary button
	if (octaveBtn.Pressed()) {

	}
	const float octave = 1.0f;
	const uint32_t newInc = calib.pitchLUT[adc.Pitch_CV] * octave;
	smoothedInc = newInc;

	// Calculate the maximum harmonic before aliasing (experimenting shows we need to truncate slightly before the nyquist frequency)
	aliasHarmonic = std::min(maxHarmonic, (uint32_t)(((float)sinLUTSize / 2.25f) / (newInc >> 16)));

	// Increment the sine read position
	uint32_t inc = std::round(newInc);
	prevIncErr = newInc - inc;				// To prevent errors compounding store the previous error to apply to the next increment
	uint32_t incMult = inc;

	float mixOut[2] = {0.0f, 0.0f};

	for (uint32_t i = 0; i < aliasHarmonic; ++i) {
		readPos[i] += incMult;
		mixOut[i & 1] += sineLUT[readPos[i] >> 16] * multipliers[i];
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

	uint32_t combPos[2] = {0, 0};
	int32_t combDir[2] = {1, 1};


	if (harmonicMode.IsLow()) {				// Mode to scale multipliers according to a cosine wave shape

		const float maxLevel = 0.15f;
		float sineScale[2] = {maxLevel, maxLevel};
		uint16_t sinePos = Additive::sinLUTSize / 4;		// Start at maximum (pi/2)

		// Calculate smoothed spread amount from pot and CV with trimmer controlling range of CV
		static constexpr float spreadMult = 10000.0f / 65535.0f;
		const float cv = std::max(61300.0f - adc.Harm_Stretch_CV, 0.0f);		// Reduce to ensure can hit zero with noise

		Smooth(multSpread, 100.0f + (adc.Harm_Stretch_Pot + (NormaliseADC(adc.Harm_Stretch_Trm) * cv)) * spreadMult, 0.99f);
		float spread = multSpread;

		static constexpr float growMult = 0.05f * (30.0f / 65535.0f);
		multGrow = multGrow * 0.95f + (adc.Harm_Warp_CV - 32768) * growMult;

		for (uint32_t i = 0; i < maxHarmonics; ++i) {
			FilterCalc(i, sineScale[i & 1], combPos[i & 1], combDir[i & 1], maxLevel);

			// Reduce the level of the last two harmonics to avoid glitching
			if (i == maxHarmonics - 2) sineScale[i & 1] *= 0.5f;
			if (i == maxHarmonics - 1) sineScale[i & 1] *= 0.25f;

			multipliers[i] = sineScale[i & 1] * (1.0f + sineLUT[sinePos]);
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
