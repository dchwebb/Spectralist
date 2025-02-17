#pragma once
#include "initialisation.h"

class LedManager {
public:
	void Init();
	void WriteRegister(uint8_t reg, uint8_t value);
	uint8_t ReadRegister(uint8_t reg);

	const uint32_t i2cAddress = 0x01;
};

extern LedManager ledManager;
