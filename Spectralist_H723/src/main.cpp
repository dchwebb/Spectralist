#include "initialisation.h"
#include "CDCHandler.h"
#include "USB.h"
#include "configManager.h"
#include "Calib.h"
//#include "Filter.h"
#include "Additive.h"

volatile uint32_t SysTickVal;
extern uint32_t SystemCoreClock;
bool SafeMode = false;				// Disables file system mounting, USB MSC drive is disabled, don't load config

volatile ADCValues __attribute__((section (".dma_buffer"))) adc;	// Store adc buffer in non-cached memory area

Config config{&calib.configSaver};		// Construct config handler with list of configSavers

extern "C" {
#include "interrupts.h"
}

bool USBDebug = false;

int main(void) {

	InitClocks();					// Configure the clock and PLL
	InitHardware();
//	filter.Init();					// Initialise look up table of filter coefficients, windows etc
	config.RestoreConfig();
	usb.Init(false);
	InitI2S();						// Initialise I2S which will start main sample interrupts

	while (1) {
		usb.cdc.ProcessCommand();	// Check for incoming USB serial commands
		config.SaveConfig();		// Save any scheduled changes
		calib.Calibrate();
		additive.IdleJobs();

#if (USB_DEBUG)
		if ((GPIOB->IDR & GPIO_IDR_ID4) == 0 && USBDebug) {
			USBDebug = false;
			usb.OutputDebug();
		}
#endif

	}
}

