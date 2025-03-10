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
enum class SampleType {Unsupported, Float32, PCM16};

static constexpr uint32_t ADC1_BUFFER_LENGTH = 6;
static constexpr uint32_t ADC2_BUFFER_LENGTH = 6;

struct ADCValues {
	uint16_t Wavetable_Pos_B_Pot;
	uint16_t Wavetable_Pos_A_Trm;
	uint16_t AudioIn;
	uint16_t WarpCV;
	uint16_t Pitch_CV;
	uint16_t WavetablePosB_CV;
	uint16_t VcaCV;
	uint16_t WavetablePosA_CV;
	uint16_t Warp_Type_Pot;
	uint16_t Wavetable_Pos_A_Pot;
	uint16_t Warp_Amt_Trm;
	uint16_t Warp_Amt_Pot;
};

extern volatile ADCValues adc;
extern GpioPin debugPin1;			// PD5: Debug
extern GpioPin debugPin2;			// PD6: Debug

void InitClocks();
void InitHardware();
void InitCache();
void InitSysTick();
void InitDisplaySPI();
void InitADC();
void InitADC1();
void InitADC2();
void InitI2S();
void InitDebugTimer();
void StartDebugTimer();
float StopDebugTimer();
void DelayMS(uint32_t ms);
void InitMDMA();
void MDMATransfer(MDMA_Channel_TypeDef* channel, const uint8_t* srcAddr, const uint8_t* destAddr, const uint32_t bytes);
void InitEncoders();
void InitOctoSPI();
void JumpToBootloader();
void Reboot();
void CheckVCA();
