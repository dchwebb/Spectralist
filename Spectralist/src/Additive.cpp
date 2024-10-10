#include "Additive.h"
#include "Calib.h"
#include <cstring>

Additive additive;

// Create sine look up table as constexpr so will be stored in flash
std::array<float, Additive::sinLUTSize> sineLUT = additive.CreateSinLUT();

Additive::Additive()
{
	readPos = {0.0f, 8192.0f, 0.0f, 12288.0f, 0.0f, 40960.0f, 0.0f, 26542.0f, 21954.0f, 18022.0f, 9011.0f, 39649.0f, 22773.0f, 45711.0f, 17694.0f, 35880.0f, 58654.0f, 43417.0f, 27688.0f, 52756.0f, 5079.0f, 20643.0f, 33259.0f, 16056.0f, 39157.0f, 53903.0f, 12124.0f, 63242.0f, 14745.0f, 36536.0f, 4259.0f, 54722.0f, 11304.0f, 45547.0f, 17530.0f, 9666.0f, 2457.0f, 57507.0f, 43581.0f, 32604.0f, 15073.0f, 64389.0f, 46858.0f, 21790.0f, 52264.0f, 10158.0f, 24903.0f, 19169.0f, 52101.0f, 61603.0f, 20643.0f, 27852.0f, 5570.0f, 44728.0f, 6717.0f, 43253.0f, 327.0f, 13434.0f, 5734.0f, 20316.0f, 17039.0f, 10158.0f, 22118.0f, 33914.0f, 42270.0f, 16384.0f, 2621.0f, 62586.0f, 29491.0f, 6553.0f, 47349.0f, 327.0f, 8192.0f, 38830.0f, 53084.0f, 23265.0f, 26050.0f, 38010.0f, 36044.0f, 53903.0f, 50626.0f, 40468.0f, 7045.0f, 46202.0f, 64225.0f, 10321.0f, 7536.0f, 20316.0f, 59965.0f, 47349.0f, 33914.0f, 327.0f, 4751.0f, 30801.0f, 10649.0f, 39321.0f, 46694.0f, 23756.0f, 35880.0f, 52920.0f, 13926.0f, 13926.0f, 32276.0f, 58327.0f, 51281.0f, 46202.0f, 48005.0f, 53575.0f, 52428.0f, 14581.0f, 38174.0f, 38338.0f, 34078.0f, 45219.0f, 42598.0f, 2785.0f, 50790.0f, 13434.0f, 46202.0f, 34897.0f, 9830.0f, 44564.0f, 31457.0f, 10485.0f, 28344.0f, 29491.0f, 38338.0f, 57671.0f, 29163.0f, 49971.0f, 50790.0f, 15892.0f, 28672.0f, 19005.0f, 58654.0f, 60456.0f, 21463.0f, 42926.0f, 1146.0f, 61767.0f, 5406.0f, 11796.0f, 59637.0f, 21135.0f, 29163.0f, 12451.0f, 64225.0f, 48168.0f, 34406.0f, 46530.0f, 59310.0f, 61931.0f, 63569.0f, 6225.0f, 59310.0f, 55541.0f, 7864.0f, 24576.0f, 46530.0f, 59146.0f, 3112.0f, 19660.0f, 42434.0f, 35553.0f, 11304.0f, 4915.0f, 7864.0f, 58327.0f, 53411.0f, 47185.0f, 25067.0f, 23265.0f, 20152.0f, 64880.0f, 36864.0f, 55869.0f, 25559.0f, 15073.0f, 29982.0f, 36372.0f, 27033.0f, 41451.0f, 39976.0f, 49643.0f, 57344.0f, 163.0f, 2621.0f, 52592.0f, 41615.0f, 20480.0f, 64061.0f, 64716.0f, 49479.0f, 58163.0f, 55050.0f, 63897.0f, 5570.0f, 47513.0f, 15892.0f, 40796.0f};
}

std::pair<float, float> Additive::ProcessSamples(Samples& samples)
{
	debug1.SetHigh();

	// Pitch calculations
	const float octave = octaveDown.IsHigh() ? 0.5 : 1.0f;
	const float newInc = calib.cfg.pitchBase * std::pow(2.0f, (float)adc.Pitch_CV * calib.cfg.pitchMult) * octave;			// for cycle length matching sample rate (48k)
	smoothedInc = 0.99 * smoothedInc + 0.01 * newInc;

	// Increment the sine read position

	float amp = 1.0f;
	float mixOutL = 0.0f;
	float mixOutR = 0.0f;
	for (uint32_t i = 0; i < 10; ++i) {
		readPos[i] += smoothedInc;
		if (readPos[i] >= Additive::sinLUTSize) {
			readPos[i] -= Additive::sinLUTSize;
		}
		mixOutL += sineLUT[(uint32_t)readPos[i]] * amp;
		amp *= 0.6;
	}
	mixOutR = mixOutL;

	return std::make_pair(mixOutL, mixOutR);
	debug1.SetLow();
}




void Additive::IdleJobs()
{

}



