#include "initialisation.h"
#include "CDCHandler.h"
#include "USB.h"
#include "configManager.h"
#include "ledManager.h"
#include "Calib.h"
#include "Additive.h"

volatile uint32_t SysTickVal;
extern uint32_t SystemCoreClock;

volatile ADCValues __attribute__((section (".dma_buffer"))) adc;	// Store adc buffer in non-cached memory area

// Initialise configurable globals here to ensure objects exist before config initialisation runs
Additive additive;
Calib calib;

Config config{&calib.configSaver, &additive.configSaver};			// Construct config handler with list of configSavers

extern "C" {
#include "interrupts.h"
}

bool USBDebug = false;

int main(void) {

	InitClocks();					// Configure the clock and PLL
	InitHardware();
	config.RestoreConfig();
	usb.Init(false);
	ledManager.Init();
	InitI2S();						// Initialise I2S which will start main sample interrupts

	while (1) {
		usb.cdc.ProcessCommand();	// Check for incoming USB serial commands
		config.SaveConfig();		// Save any scheduled changes
		calib.Calibrate();			// Calibration state machine
		additive.IdleJobs();		// Calculate levels of sine wave multipliers

#if (USB_DEBUG)
		if ((GPIOB->IDR & GPIO_IDR_ID4) == 0 && USBDebug) {
			USBDebug = false;
			usb.OutputDebug();
		}
#endif

	}
}

