#pragma once
#include "initialisation.h"
#include "AudioCodec.h"
#include <tuple>
#include <array>

class Additive {
public:
	Additive();
	static constexpr uint32_t harmonics = 200;
	std::array<float, harmonics> readPos;

	std::pair<float, float> ProcessSamples(Samples& samples);
	void IdleJobs();

	static constexpr uint32_t sinLUTSize = 65536;
	auto CreateSinLUT()									// constexpr function to generate LUT in Flash
	{
		std::array<float, sinLUTSize> array {};					// Create one extra entry to simplify interpolation
		for (uint32_t s = 0; s < sinLUTSize; ++s){
			array[s] = std::sin(s * 2.0f * std::numbers::pi / sinLUTSize);
		}
		return array;
	}
private:

	float smoothedInc = 0.0f;

	GpioPin octaveDown	{GPIOD, 6, GpioPin::Type::Input};
	GpioPin debug1		{GPIOC, 10, GpioPin::Type::Output};

};

extern Additive additive;
