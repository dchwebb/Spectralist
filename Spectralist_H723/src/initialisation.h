#pragma once

#include "stm32h7xx.h"
#include <algorithm>
#include <cstdlib>
#include <cmath>
#include "GpioPin.h"

extern volatile uint32_t SysTickVal;
extern bool SafeMode;
extern bool modeSpectralist;
extern bool vcaConnected;			// Temporary hack as current hardware does not have VCA normalled correctly

static constexpr uint32_t sysTickInterval = 1000;					// Set in uS so 1000uS = 1ms
constexpr uint32_t sampleRate = 48000;
static constexpr uint32_t adcMax = 65536;

static constexpr uint32_t ADC1_BUFFER_LENGTH = 6;
static constexpr uint32_t ADC2_BUFFER_LENGTH = 6;
static constexpr uint32_t ADC3_BUFFER_LENGTH = 3;

struct ADCValues {
	uint16_t Filter_A_Pot;
	uint16_t Filter_A_Trm;
	uint16_t Pitch_CV;
	uint16_t Filter_Slope_CV;
	uint16_t Harm_Warp_CV;
	uint16_t Harm_Stretch_CV;
	uint16_t Filter_B_CV;
	uint16_t Harm_Warp_Pot;
	uint16_t Filter_Slope_Pot;
	uint16_t Filter_B_Pot;
	uint16_t Harm_Stretch_Trm;
	uint16_t Harm_Stretch_Pot;
	uint16_t Filter_B_Trm;
	uint16_t Pan_CV;
	uint16_t Filter_A_CV;
};

extern volatile ADCValues adc;
extern GpioPin debugPin1;			// PD5: Debug
extern GpioPin debugPin2;			// PD6: Debug

void InitClocks();
void InitHardware();
void InitCache();
void InitSysTick();
void InitADC();
void InitADC1();
void InitADC2();
void InitADC3();
void InitI2S();
void InitI2C();
void InitCordic();
void InitPWMTimer();
void InitDebugTimer();
void StartDebugTimer();
float StopDebugTimer();
void DelayMS(uint32_t ms);
void JumpToBootloader();
void Reboot();
void CheckVCA();
