#pragma once
#include "initialisation.h"

#define I2C_DMA_Working ((I2C5->ISR & I2C_SR_TXE) == 0)

class LedManager {
public:
	void Init();
	void WriteRegister(uint8_t reg, uint8_t value);
	uint8_t ReadRegister(uint8_t reg);
	void DMASend();

	const uint32_t i2cAddress = 0x01;

	static constexpr uint32_t ledCount = 25;			// 24 leds plus one byte for the command register
	static uint8_t leds[ledCount];		// declared static to allow placement in non cached RAM
};

extern LedManager ledManager;
