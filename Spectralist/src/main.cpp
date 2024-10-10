#include "initialisation.h"
#include "USB.h"
#include "AudioCodec.h"
#include "Additive.h"
#include "Calib.h"

volatile uint32_t SysTickVal;
volatile uint32_t outputUSB = 0;		// USB debugging
volatile ADCValues adc;

extern "C" {
#include "interrupts.h"
}

Config config{&calib.configSaver};		// Construct config handler with list of configSavers


extern uint32_t SystemCoreClock;
int main(void)
{
	SystemInit();						// Activates floating point coprocessor and resets clock
	InitSystemClock();					// Configure the clock and PLL
	InitHardware();
	config.RestoreConfig();
	audioCodec.Init();
	usb.InitUSB();

	while (1) {
#if (USB_DEBUG)
		if (SysTickVal - outputUSB > 1000 && (GPIOC->IDR & GPIO_IDR_ID13) == GPIO_IDR_ID13) {
			usb.OutputDebug();
			outputUSB = SysTickVal;
		}
#endif
		usb.cdc.ProcessCommand();
		calib.Calibrate();

	}
}

