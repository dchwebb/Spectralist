#pragma once

#include "stm32h563xx.h"
#include "GpioPin.h"
#include <numbers>
#include <cmath>

extern volatile uint32_t SysTickVal;
static constexpr uint32_t sysTick = 1000;						// 1ms
static constexpr uint32_t sampleRate = 48000;
static constexpr float inverseSampleRate  = 1.0f / (float)sampleRate;
static constexpr float pi = std::numbers::pi_v<float>;
static constexpr float pi_x_2 = pi * 2.0f;
static constexpr uint32_t adcMax = 4095;

#ifndef M_PI_4
#define M_PI_4 0.78539816339744830962f
#define M_1_PI 0.63661977236758134308f
#endif

struct ADCValues {
	uint16_t effectMix;               // effectMix;
	uint16_t effectLFORange;          // effectLFORange;
	uint16_t effectLFOSpeed;          // effectLFOSpeed;
	uint16_t Pitch_CV;       // effectLFOBaseFreq;
	uint16_t effectRegen;             // effectRegen;
	uint16_t delayTimePot;            // delayTimePot;
	uint16_t delayTimeCV;             // delayTimeCV;
	uint16_t delayFeedback;           // delayFeedback;
	uint16_t delayFilter;             // delayFilter;
	uint16_t delayMix;                // delayMix;
};
extern volatile ADCValues adc;


struct DMALinkedList {
	uint32_t TR1;
	uint32_t TR2;
	uint32_t BR1;
	uint32_t SAR;
	uint32_t DAR;
	uint32_t LLR;
};


void InitSystemClock();
void InitSAI();
void InitHardware();
void InitAudioCodec();
void InitSysTick();
void InitMPU();
void InitADC2(volatile uint16_t* buffer, uint16_t channels);
void InitCordic();
void InitPWMTimer();
