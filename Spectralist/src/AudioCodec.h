#pragma once

#include "initialisation.h"

struct Samples {
	float ch[4];
};

class AudioCodec {
public:
	void Init();
	uint8_t ReadData(uint16_t address);
	void WriteData(uint16_t address, uint8_t data);
	void Interrupt();

	Samples dataIn;

	bool leftRight = true;				// to keep count of which channel is being received
	bool outputDone = false;			// To alert the main loop it can run idle jobs

private:
	int32_t Denormalise(float x);		// Apply Tanh limiter and convert from 0-1 to 32 bit

	struct __attribute__((__packed__)) Command {
		enum CommandType : uint8_t {Read  = 0x43, Write = 0xC3, activateSPI = 0xDE};
		enum RegisterAddress : uint16_t {PowerManagement = 0x00, AudioInterfaceFormat = 0x01, AnalogInput = 0x0B, AnalogFilter = 0x0A, SpiMode = 0xADDA};
		Command(CommandType rw, uint16_t address, uint8_t data) : data(data), address(address), cmdCode(rw) {}

		uint8_t data;
		uint16_t address;
		CommandType cmdCode;
	};
	void SendCmd(Command cmd);

	GpioPin pdnPin{GPIOD, 15, GpioPin::Type::Output};		// PDN pin is used to bring Codec out of reset
	GpioPin debugPin{GPIOC, 10, GpioPin::Type::Output};		// Timing Debug PIn

	static constexpr float normalise32Bit = 1.0f / std::pow(2, 31);
	static constexpr float denormalise32Bit = std::pow(2, 31);
};

extern AudioCodec audioCodec;
