#pragma once
#include "initialisation.h"
#include "configManager.h"
#include <tuple>
#include <array>

class Additive {
public:
	Additive();

	static constexpr uint32_t maxHarmonics = 300;
	uint32_t readPos[maxHarmonics] = {0,536870912,0,805306368,0,2684354560,0,1739461754,1438814044,1181116006,590558003,2598455214,1492501135,2995739688,1159641169,2351494594,3843995729,2845415833,1814623682,3457448673,332859965,1352914698,2179695902,1052266987,2566242959,3532610600,794568949,4144643440,966367641,2394444267,279172874,3586297692,740881858,2985002270,1148903751,633507676,161061273,3768833802,2856153251,2136746229,987842478,4219805368,3070901616,1428076625,3425236418,665719930,1632087572,1256277934,3414499000,4037269258,1352914698,1825361100,365072220,2931315179,440234147,2834678415,21474836,880468295,375809638,1331439861,1116691496,665719930,1449551462,2222645575,2770253905,1073741824,171798691,4101693767,1932735283,429496729,3103113871,21474836,536870912,2544768122,3478923509,1524713390,1707249500,2491081031,2362232012,3532610600,3317862236,2652142305,461708984,3027951943,4209067950,676457349,493921239,1331439861,3929895075,3103113871,2222645575,21474836,311385128,2018634629,697932185,2576980377,3060164198,1556925644,2351494594,3468186091,912680550,912680550,2115271393,3822520893,3360811909,3027951943,3146063544,3511135764,3435973836,955630223,2501818449,2512555868,2233382993,2963527434,2791728742,182536110,3328599654,880468295,3027951943,2287070085,644245094,2920577761,2061584302,687194767,1857573355,1932735283,2512555868,3779571220,1911260446,3274912563,3328599654,1041529569,1879048192,1245540515,3843995729,3962107330,1406601789,2813203578,75161927,4048006676,354334801,773094113,3908420239,1385126952,1911260446,816043786,4209067950,3156800962,2254857830,3049426780,3886945402,4058744094,4166118277,408021893,3886945402,3639984783,515396075,1610612736,3049426780,3876207984,204010946,1288490188,2780991324,2330019758,740881858,322122547,515396075,3822520893,3500398346,3092376453,1642824990,1524713390,1320702443,4252017623,2415919104,3661459619,1675037245,987842478,1964947537,2383706849,1771674009,2716566814,2619930050,3253437726,3758096384,10737418,171798691,3446711255,2727304232,1342177280,4198330531,4241280204,3242700308,3811783475,3607772528,4187593113,365072220,3113851289,1041529569,2673617141};

	void CalcSample();
	void IdleJobs();
	static void UpdateConfig();

	struct {
		bool octaveDown = false;
	} cfg;

	ConfigSaver configSaver = {
		.settingsAddress = &cfg,
		.settingsSize = sizeof(cfg),
		.validateSettings = UpdateConfig
	};

	static constexpr uint32_t sinLUTSize = 32768;
	static constexpr uint32_t sinLUTBits = log2(sinLUTSize);
	static constexpr uint32_t sinLUTShift32 = 32 - sinLUTBits;		// Size of right shift of 32 uint to map to sine LUT size
	static constexpr uint32_t sinLUTShift16 = 16 - sinLUTBits;		// Size of right shift of 16 uint to map to sine LUT size

	constexpr auto CreateSinLUT()
	{
		std::array<float, sinLUTSize> array {};
		for (uint32_t s = 0; s < sinLUTSize; ++s){
			array[s] = std::sin(s * 2.0f * std::numbers::pi / sinLUTSize);
		}
		return array;
	}

private:
	float FastTanh(const float x);
	void UpdateLEDs();
	inline void FilterCalc(uint32_t pos, float& scale, uint32_t& combPos, int32_t& combDir, float maxLevel);
	inline void Smooth(float& currentValue, const float newValue, const float fraction);
	float EqualPowerCrossfade(const float mix1, const float sample1, const float sample2);

	float outputSamples[2] = {0.0f, 0.0f};							// Preprepared samples sent to DAC on interrupt
	uint32_t aliasHarmonic;

	static constexpr float scaleOutput = -std::pow(2.0f, 31.0f);	// Multiple to convert -1.0 - 1.0 float to 32 bit int and invert

	enum class Filter {LP, HP, BP, Comb, count} filterType = Filter::LP;
	int32_t filterTypeVal = 0;										// Used for setting hysteresis on filter type
	float filterStart[2] = {0.0f, 0.0f};
	float filterSlope = 0.0f;

	float multSpread = 0.0f;
	float multGrow = 0.0f;

	float multipliers[maxHarmonics];

	static inline float NormaliseADC(uint16_t adcVal)
	{
		static constexpr float adcDivider = 1.0f / 55000.0f;		// reduce divider to allow maximum of slightly over 1.0f
		return adcDivider * adcVal;
	}

	GpioPin harmonicMode	{GPIOE, 3, GpioPin::Type::InputPulldown};
	GpioPin filterMode		{GPIOE, 2, GpioPin::Type::InputPulldown};
	GpioPin octaveLED		{GPIOD, 14, GpioPin::Type::Output};

	Btn octaveBtn = {{GPIOE, 4, GpioPin::Type::InputPullup}, 0, 0};


	volatile uint32_t& filterLED_A = TIM4->CCR4;					// Values from 0 to 4095 for brightness
	volatile uint32_t& filterLED_B = TIM4->CCR2;
};

extern Additive additive;
