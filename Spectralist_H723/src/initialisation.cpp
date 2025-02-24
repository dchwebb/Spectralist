#include "stm32h723xx.h"
#include "initialisation.h"
#include "Calib.h"
#include <cstring>

GpioPin debugPin1	{GPIOD, 6, GpioPin::Type::Output};			// PD5: Debug (also UART_TX)
GpioPin debugPin2	{GPIOD, 5, GpioPin::Type::Output};			// PD6: Debug (also UART_RX)
GpioPin debugPin3	{GPIOA, 15, GpioPin::Type::Output};			// PA15: Debug (also SPI_NSS)

// Clock overview:
// I2S 				PLL2 P 				61.44 MHz
// ADC		 		Peripheral Clock 	8 MHz

// Main clock 8MHz (HSE) / 2 (M) * 100 (N) / 1 (P) = 400MHz
#define PLL_M1 2
#define PLL_N1 100
#define PLL_P1 1

// PPL2P used for I2S: 		8MHz / 5 (M) * 192 (N) / 5 (P) = 61.44 MHz
#define PLL_M2 5
#define PLL_N2 192
#define PLL_P2 5
#define PLL_R2 2


void InitClocks()
{
	RCC->APB4ENR |= RCC_APB4ENR_SYSCFGEN;			// Enable System configuration controller clock

	// Voltage scaling - see Datasheet page 91. VOS0 > 520MHz; VOS1 > 400MHz; VOS2 > 300MHz; VOS3 < 170MHz
	PWR->D3CR &= ~PWR_D3CR_VOS;						// Configure voltage scaling level 0 (Highest)
	while ((PWR->D3CR & PWR_D3CR_VOSRDY) == 0);	// Check Voltage ready 1= Ready, voltage level at or above VOS selected level

	RCC->CR |= RCC_CR_HSEON;						// Turn on external oscillator
	while ((RCC->CR & RCC_CR_HSERDY) == 0);			// Wait till HSE is ready

	// Clock source to High speed external and main (M) dividers
	RCC->PLLCKSELR = RCC_PLLCKSELR_PLLSRC_HSE |
	                 PLL_M1 << RCC_PLLCKSELR_DIVM1_Pos;

	// PLL 1 dividers
	RCC->PLL1DIVR = (PLL_N1 - 1) << RCC_PLL1DIVR_N1_Pos |
			        (PLL_P1 - 1) << RCC_PLL1DIVR_P1_Pos;

	RCC->PLLCFGR |= RCC_PLLCFGR_PLL1RGE_1;			// 01: The PLL1 input (ref1_ck) between 2 and 4 MHz (Will be 4MHz for 8MHz clock divided by 2)
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLL1VCOSEL;		// 0: Wide VCO range: 192 to 836 MHz (default); 1: Medium VCO range: 150 to 420 MHz
	RCC->PLLCFGR |= RCC_PLLCFGR_DIVP1EN;			// Enable P divider output

	RCC->CR |= RCC_CR_PLL1ON;						// Turn on main PLL
	while ((RCC->CR & RCC_CR_PLL1RDY) == 0);		// Wait till PLL is ready

	// PLL 2 dividers
	RCC->PLLCKSELR |= PLL_M2 << RCC_PLLCKSELR_DIVM2_Pos;

	RCC->PLL2DIVR = (PLL_N2 - 1) << RCC_PLL2DIVR_N2_Pos |
					(PLL_P2 - 1) << RCC_PLL2DIVR_P2_Pos |
					(PLL_R2 - 1) << RCC_PLL2DIVR_R2_Pos;

	RCC->PLLCFGR |= RCC_PLLCFGR_PLL2RGE_1 |			// 01: The PLL2 input (ref1_ck) clock range frequency is between 4 and 8 MHz
					RCC_PLLCFGR_DIVP2EN |			// Enable P divider output
					RCC_PLLCFGR_DIVR2EN;			// Enable R divider output

	RCC->CR |= RCC_CR_PLL2ON;						// Turn on PLL 2
	while ((RCC->CR & RCC_CR_PLL2RDY) == 0);		// Wait till PLL is ready

	// Peripheral scalers
	RCC->D1CCIPR |= RCC_D1CCIPR_CKPERSEL_1;			// Peripheral clock to HSE (8MHz)
	RCC->D1CFGR |= RCC_D1CFGR_HPRE_DIV2;			// D1 domain AHB prescaler - divide 500MHz by 2 for 250MHz - this is then divided for all APB clocks below
	RCC->D1CFGR |= RCC_D1CFGR_D1PPRE_DIV2;			// APB3 Clocks
	RCC->D2CFGR |= RCC_D2CFGR_D2PPRE1_DIV2;			// APB1 Clocks
	RCC->D2CFGR |= RCC_D2CFGR_D2PPRE2_DIV2;			// APB2 Clocks
	RCC->D3CFGR |= RCC_D3CFGR_D3PPRE_DIV2;			// APB4 Clocks

	// By default Flash latency is set to 7 wait states - Note values in table are half system clock rates
	FLASH->ACR = FLASH_ACR_WRHIGHFREQ |
				(FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_4WS;
	while ((FLASH->ACR & FLASH_ACR_LATENCY_Msk) != FLASH_ACR_LATENCY_4WS);

	RCC->CFGR |= RCC_CFGR_SW_PLL1;					//3 System clock switch: 011: PLL1 selected as system clock
	while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != (RCC_CFGR_SW_PLL1 << RCC_CFGR_SWS_Pos));		// Wait until PLL has been selected as system clock source


	SystemCoreClockUpdate();						// Update SystemCoreClock (system clock frequency) derived from settings of oscillators, prescalers and PLL
}


void InitHardware()
{
	InitSysTick();
	InitCache();
	InitADC();
	InitPWMTimer();
	InitCordic();
}


void InitCache()
{
	// Use the Memory Protection Unit (MPU) to set up a region of memory with data caching disabled for use with DMA buffers
	MPU->RNR = 0;									// Memory region number
	extern uint32_t _dma_addr;						// Get the start of the dma buffer from the linker
	MPU->RBAR = reinterpret_cast<uint32_t>(&_dma_addr);	// Store the address of the ADC_array into the region base address register

	MPU->RASR = (0b11  << MPU_RASR_AP_Pos)   |		// All access permitted
				(0b001 << MPU_RASR_TEX_Pos)  |		// Type Extension field: See truth table on p228 of Cortex M7 programming manual
				(1     << MPU_RASR_S_Pos)    |		// Shareable: provides data synchronization between bus masters. Eg a processor with a DMA controller
				(0     << MPU_RASR_C_Pos)    |		// Cacheable
				(0     << MPU_RASR_B_Pos)    |		// Bufferable (ignored for non-cacheable configuration)
				(14    << MPU_RASR_SIZE_Pos) |		// 32KB - (size is log 2(mem size) - 1 ie 2^15 = 32k)
				(1     << MPU_RASR_ENABLE_Pos);		// Enable MPU region


	MPU->CTRL |= (1 << MPU_CTRL_PRIVDEFENA_Pos) |	// Enable PRIVDEFENA - this allows use of default memory map for memory areas other than those specific regions defined above
				 (1 << MPU_CTRL_ENABLE_Pos);		// Enable the MPU

	// Enable data and instruction caches
	SCB_EnableDCache();
	SCB_EnableICache();
}


void InitSysTick()
{
	SysTick_Config(SystemCoreClock / sysTickInterval);		// gives 1ms
	NVIC_SetPriority(SysTick_IRQn, 0);
}


void InitAdcPins(ADC_TypeDef* ADC_No, std::initializer_list<uint8_t> channels)
{
	uint8_t sequence = 1;

	for (auto channel: channels) {
		// NB reset mode of GPIO pins is 0b11 = analog mode so shouldn't need to change
		ADC_No->PCSEL_RES0 |= 1 << channel;					// ADC channel preselection register

		// Set conversion sequence to order ADC channels are passed to this function
		if (sequence < 5) {
			ADC_No->SQR1 |= channel << (sequence * 6);
		} else if (sequence < 10) {
			ADC_No->SQR2 |= channel << ((sequence - 5) * 6);
		} else if (sequence < 15)  {
			ADC_No->SQR3 |= channel << ((sequence - 10) * 6);
		} else {
			ADC_No->SQR4 |= channel << ((sequence - 15) * 6);
		}

		// 000: 1.5 ADC clock cycles; 001: 2.5 cycles; 010: 8.5 cycles;	011: 16.5 cycles; 100: 32.5 cycles; 101: 64.5 cycles; 110: 387.5 cycles; 111: 810.5 cycles
		if (channel < 10)
			ADC_No->SMPR1 |= 0b010 << (3 * channel);
		else
			ADC_No->SMPR2 |= 0b010 << (3 * (channel - 10));

		sequence++;
	}
}


void InitADC()
{
	// Settings used for both ADC1 and ADC2

	// Configure clocks
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN;			// GPIO port clock
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN;
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOCEN;

	RCC->AHB1ENR |= RCC_AHB1ENR_ADC12EN;
	RCC->AHB4ENR |= RCC_AHB4ENR_ADC3EN;
	RCC->D3CCIPR |= RCC_D3CCIPR_ADCSEL_1;			// ADC clock selection: 10: per_ck clock (HSE 8MHz)
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

	ADC12_COMMON->CCR |= ADC_CCR_PRESC_0;			// Set prescaler to ADC clock divided by 2 (if 8MHz = 4MHz)
	ADC3_COMMON->CCR |= ADC_CCR_PRESC_0;			// Set prescaler to ADC clock divided by 2 (if 8MHz = 4MHz)

	InitADC1();
	InitADC2();
	InitADC3();
}


/*-----------------------------------------------------------------------------------------------------------------
ADC1:
	PA0 ADC1_INP16 Filter_A_Pot
	PA1 ADC1_INP17 Filter_A_Trm
	PA2 ADC1_INP14 Pitch_CV_Scaled
	PA3 ADC1_INP15 Filter_Slope_CV
	PA4 ADC1_INP18 Harm_Warp_CV
	PA5 ADC1_INP19 Harm_Stretch_CV
ADC2:
	PA6 ADC12_INP3 Filter_B_CV
	PA7 ADC12_INP7 Harm_Stretch_Pot
	PB0 ADC12_INP9 Filter_Slope_Pot
	PB1 ADC12_INP5 Filter_B_Pot
	PC4 ADC12_INP4 Harm_Stretch_Trm
	PC5 ADC12_INP8 Harm_Warp_Pot
ADC3:
	PC0 ADC123_INP10 Filter_B_Trm
	PC1	ADC123_INP11 Pan_CV
	PC2_C ADC3_INP0 Filter_A_CV

*/

void InitADC1()
{
	// Initialize ADC peripheral
	DMA1_Stream1->CR &= ~DMA_SxCR_EN;
	DMA1_Stream1->CR |= DMA_SxCR_CIRC;				// Circular mode to keep refilling buffer
	DMA1_Stream1->CR |= DMA_SxCR_MINC;				// Memory in increment mode
	DMA1_Stream1->CR |= DMA_SxCR_PSIZE_0;			// Peripheral size: 8 bit; 01 = 16 bit; 10 = 32 bit
	DMA1_Stream1->CR |= DMA_SxCR_MSIZE_0;			// Memory size: 8 bit; 01 = 16 bit; 10 = 32 bit
	DMA1_Stream1->CR |= DMA_SxCR_PL_0;				// Priority: 00 = low; 01 = Medium; 10 = High; 11 = Very High

	DMA1_Stream1->FCR &= ~DMA_SxFCR_FTH;			// Disable FIFO Threshold selection
	DMA1->LIFCR = 0x3F << DMA_LIFCR_CFEIF1_Pos;		// clear all five interrupts for this stream

	DMAMUX1_Channel1->CCR |= 9; 					// DMA request MUX input 9 = adc1_dma (See p.676)
	DMAMUX1_ChannelStatus->CFR |= DMAMUX_CFR_CSOF1; // Channel 1 Clear synchronization overrun event flag

	ADC1->CR &= ~ADC_CR_DEEPPWD;					// Deep powDMA1_Stream2own: 0: ADC not in deep-power down	1: ADC in deep-power-down (default reset state)
	ADC1->CR |= ADC_CR_ADVREGEN;					// Enable ADC internal voltage regulator

	// Wait until voltage regulator settled
	volatile uint32_t wait_loop_index = (SystemCoreClock / (100000UL * 2UL));
	while (wait_loop_index != 0UL) {
		wait_loop_index--;
	}
	while ((ADC1->CR & ADC_CR_ADVREGEN) != ADC_CR_ADVREGEN) {}

	ADC1->CFGR |= ADC_CFGR_CONT;					// 1: Continuous conversion mode for regular conversions
	ADC1->CFGR |= ADC_CFGR_OVRMOD;					// Overrun Mode 1: ADC_DR register is overwritten with the last conversion result when an overrun is detected.
	ADC1->CFGR |= ADC_CFGR_DMNGT;					// Data Management configuration 01: DMA One Shot Mode selected, 11: DMA Circular Mode selected

	//00: ADC clock ≤ 6.25 MHz; 01: 6.25 MHz < clk ≤ 12.5 MHz; 10: 12.5 MHz < clk ≤ 25.0 MHz; 11: 25.0 MHz < clock ≤ 50.0 MHz
	ADC1->CR |= ADC_CR_BOOST_0;
	ADC1->SQR1 |= (ADC1_BUFFER_LENGTH - 1);			// For scan mode: set number of channels to be converted

	// Start calibration
	ADC1->CR |= ADC_CR_ADCALLIN;					// Activate linearity calibration (as well as offset calibration)
	ADC1->CR |= ADC_CR_ADCAL;
	while ((ADC1->CR & ADC_CR_ADCAL) == ADC_CR_ADCAL);


	/*--------------------------------------------------------------------------------------------
	Configure ADC Channels to be converted:
	PA0 ADC1_INP16 Filter_A_Pot
	PA1 ADC1_INP17 Filter_A_Trm
	PA2 ADC1_INP14 Pitch_CV_Scaled
	PA3 ADC1_INP15 Filter_Slope_CV
	PA4 ADC1_INP18 Harm_Warp_CV
	PA5 ADC1_INP19 Harm_Stretch_CV
	*/
	InitAdcPins(ADC1, {16, 17, 14, 15, 18, 19});

	// Enable ADC
	ADC1->CR |= ADC_CR_ADEN;
	while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) {}

	DMAMUX1_ChannelStatus->CFR |= DMAMUX_CFR_CSOF1; // Channel 1 Clear synchronization overrun event flag
	DMA1->LIFCR = 0x3F << DMA_LIFCR_CFEIF1_Pos;		// clear all five interrupts for this stream

	DMA1_Stream1->NDTR |= ADC1_BUFFER_LENGTH;		// Number of data items to transfer (ie size of ADC buffer)
	DMA1_Stream1->PAR = (uint32_t)(&(ADC1->DR));	// Configure the peripheral data register address 0x40022040
	DMA1_Stream1->M0AR = (uint32_t)(&adc);			// Configure the memory address (note that M1AR is used for double-buffer mode) 0x24000040

	DMA1_Stream1->CR |= DMA_SxCR_EN;				// Enable DMA and wait
	wait_loop_index = (SystemCoreClock / (100000UL * 2UL));
	while (wait_loop_index != 0UL) {
	  wait_loop_index--;
	}

	ADC1->CR |= ADC_CR_ADSTART;						// Start ADC
}


void InitADC2()
{
	// Initialize ADC peripheral
	DMA1_Stream2->CR &= ~DMA_SxCR_EN;
	DMA1_Stream2->CR |= DMA_SxCR_CIRC;				// Circular mode to keep refilling buffer
	DMA1_Stream2->CR |= DMA_SxCR_MINC;				// Memory in increment mode
	DMA1_Stream2->CR |= DMA_SxCR_PSIZE_0;			// Peripheral size: 8 bit; 01 = 16 bit; 10 = 32 bit
	DMA1_Stream2->CR |= DMA_SxCR_MSIZE_0;			// Memory size: 8 bit; 01 = 16 bit; 10 = 32 bit
	DMA1_Stream2->CR |= DMA_SxCR_PL_0;				// Priority: 00 = low; 01 = Medium; 10 = High; 11 = Very High

	DMA1_Stream2->FCR &= ~DMA_SxFCR_FTH;			// Disable FIFO Threshold selection
	DMA1->LIFCR = 0x3F << DMA_LIFCR_CFEIF2_Pos;		// clear all five interrupts for this stream

	DMAMUX1_Channel2->CCR |= 10; 					// DMA request MUX input 10 = adc2_dma (See p.676)
	DMAMUX1_ChannelStatus->CFR |= DMAMUX_CFR_CSOF2; // Channel 2 Clear synchronization overrun event flag

	ADC2->CR &= ~ADC_CR_DEEPPWD;					// Deep power down: 0: ADC not in deep-power down	1: ADC in deep-power-down (default reset state)
	ADC2->CR |= ADC_CR_ADVREGEN;					// Enable ADC internal voltage regulator

	// Wait until voltage regulator settled
	volatile uint32_t wait_loop_index = (SystemCoreClock / (100000UL * 2UL));
	while (wait_loop_index != 0UL) {
		wait_loop_index--;
	}
	while ((ADC2->CR & ADC_CR_ADVREGEN) != ADC_CR_ADVREGEN) {}

	ADC2->CFGR |= ADC_CFGR_CONT;					// 1: Continuous conversion mode for regular conversions
	ADC2->CFGR |= ADC_CFGR_OVRMOD;					// Overrun Mode 1: ADC_DR register is overwritten with the last conversion result when an overrun is detected.
	ADC2->CFGR |= ADC_CFGR_DMNGT;					// Data Management configuration 11: DMA Circular Mode selected

	// Boost mode 1: Boost mode on. Must be used when ADC clock > 20 MHz.
	ADC2->CR |= ADC_CR_BOOST_0;						// Note this sets reserved bit according to SFR - HAL has notes about silicon revision
	ADC2->SQR1 |= (ADC2_BUFFER_LENGTH - 1);			// For scan mode: set number of channels to be converted

	// Start calibration
	ADC2->CR |= ADC_CR_ADCALLIN;					// Activate linearity calibration (as well as offset calibration)
	ADC2->CR |= ADC_CR_ADCAL;
	while ((ADC2->CR & ADC_CR_ADCAL) == ADC_CR_ADCAL) {};

	/* Configure ADC Channels to be converted:
	PA6 ADC12_INP3 Filter_B_CV
	PA7 ADC12_INP7 Harm_Warp_Pot
	PB0 ADC12_INP9 Filter_Slope_Pot
	PB1 ADC12_INP5 Filter_B_Pot
	PC4 ADC12_INP4 Harm_Stretch_Trm
	PC5 ADC12_INP8 Harm_Stretch_Pot
	*/
	InitAdcPins(ADC2, {3, 7, 9, 5, 4, 8});

	// Enable ADC
	ADC2->CR |= ADC_CR_ADEN;
	while ((ADC2->ISR & ADC_ISR_ADRDY) == 0) {}

	DMAMUX1_ChannelStatus->CFR |= DMAMUX_CFR_CSOF2; // Channel 2 Clear synchronization overrun event flag
	DMA1->LIFCR = 0x3F << DMA_LIFCR_CFEIF2_Pos;		// clear all five interrupts for this stream

	DMA1_Stream2->NDTR |= ADC2_BUFFER_LENGTH;		// Number of data items to transfer (ie size of ADC buffer)
	DMA1_Stream2->PAR = reinterpret_cast<uint32_t>(&(ADC2->DR));	// Configure the peripheral data register address 0x40022040
	DMA1_Stream2->M0AR = reinterpret_cast<uint32_t>(&adc.Filter_B_CV);		// Configure the memory address (note that M1AR is used for double-buffer mode) 0x24000040

	DMA1_Stream2->CR |= DMA_SxCR_EN;				// Enable DMA and wait
	wait_loop_index = (SystemCoreClock / (100000UL * 2UL));
	while (wait_loop_index != 0UL) {
		wait_loop_index--;
	}

	ADC2->CR |= ADC_CR_ADSTART;						// Start ADC
}


void InitADC3()
{
	// Initialize ADC peripheral - NB ADC 3 is only 12 bit whilst ADC1/2 are 16 bit (values are left shifted by 4 for consistency)
	DMA1_Stream3->CR &= ~DMA_SxCR_EN;
	DMA1_Stream3->CR |= DMA_SxCR_CIRC;				// Circular mode to keep refilling buffer
	DMA1_Stream3->CR |= DMA_SxCR_MINC;				// Memory in increment mode
	DMA1_Stream3->CR |= DMA_SxCR_PSIZE_0;			// Peripheral size: 8 bit; 01 = 16 bit; 10 = 32 bit
	DMA1_Stream3->CR |= DMA_SxCR_MSIZE_0;			// Memory size: 8 bit; 01 = 16 bit; 10 = 32 bit
	DMA1_Stream3->CR |= DMA_SxCR_PL_0;				// Priority: 00 = low; 01 = Medium; 10 = High; 11 = Very High

	DMA1_Stream3->FCR &= ~DMA_SxFCR_FTH;			// Disable FIFO Threshold selection
	DMA1->LIFCR = 0x3F << DMA_LIFCR_CFEIF3_Pos;		// clear all five interrupts for this stream

	DMAMUX1_Channel3->CCR |= 115; 					// DMA request MUX input 115 = adc3_dma (See p.676)
	DMAMUX1_ChannelStatus->CFR |= DMAMUX_CFR_CSOF3; // Clear synchronization overrun event flag

	ADC3->CR &= ~ADC_CR_DEEPPWD;					// Deep power down: 0: ADC not in deep-power down	1: ADC in deep-power-down (default reset state)
	ADC3->CR |= ADC_CR_ADVREGEN;					// Enable ADC internal voltage regulator

	// Wait until voltage regulator settled: 5-10uS. Datasheet p 157
	volatile uint32_t wait_loop_index = (SystemCoreClock / (100000UL * 2UL));
	while (wait_loop_index != 0UL) {
		wait_loop_index--;
	}
	while ((ADC3->CR & ADC_CR_ADVREGEN) != ADC_CR_ADVREGEN) {}

	ADC3->CFGR |= ADC_CFGR_CONT;					// 1: Continuous conversion mode for regular conversions
	ADC3->CFGR |= ADC_CFGR_OVRMOD;					// Overrun Mode 1: ADC_DR register is overwritten with the last conversion result when an overrun is detected.
	ADC3->CFGR |= ADC_CFGR_DMNGT;					// Data Management configuration 11: DMA Circular Mode selected
	ADC3->CFGR |= ADC3_CFGR_ALIGN;					// Left align data so that 12 bit value is converted to 16 bit

	// Boost mode 1: Boost mode on. Must be used when ADC clock > 20 MHz.
	ADC3->CR |= ADC_CR_BOOST_0;						// Note this sets reserved bit according to SFR - HAL has notes about silicon revision
	ADC3->SQR1 |= (ADC3_BUFFER_LENGTH - 1);			// For scan mode: set number of channels to be converted

	// Start calibration
	ADC3->CR |= ADC_CR_ADCALLIN;					// Activate linearity calibration (as well as offset calibration)
	ADC3->CR |= ADC_CR_ADCAL;
	while ((ADC3->CR & ADC_CR_ADCAL) == ADC_CR_ADCAL) {};

	/* Configure ADC Channels to be converted:
	PC0 ADC123_INP10 Filter_B_Trm
	PC1	ADC123_INP11 Pan_CV
	PC2_C ADC3_INP0 Filter_A_CV
	*/
	InitAdcPins(ADC3, {10, 11, 0});

	// Enable ADC
	ADC3->ISR |= ADC_ISR_ADRDY;						// Clear the ADC Ready bit
	ADC3->CR |= ADC_CR_ADEN;
	while ((ADC3->ISR & ADC_ISR_ADRDY) == 0) {
		if ((ADC3->CR & ADC_CR_ADEN) == 0) {		// Not sure why but ADC3 does not turn on immediately
			ADC3->CR |= ADC_CR_ADEN;
		}
	}

	DMAMUX1_ChannelStatus->CFR |= DMAMUX_CFR_CSOF3; // Channel 3 Clear synchronization overrun event flag
	DMA1->LIFCR = 0x3F << DMA_LIFCR_CFEIF3_Pos;		// clear all five interrupts for this stream

	DMA1_Stream3->NDTR |= ADC3_BUFFER_LENGTH;		// Number of data items to transfer (ie size of ADC buffer)
	DMA1_Stream3->PAR = reinterpret_cast<uint32_t>(&(ADC3->DR));	// Configure the peripheral data register address 0x40022040
	DMA1_Stream3->M0AR = reinterpret_cast<uint32_t>(&adc.Filter_B_Trm);		// Configure the memory address (note that M1AR is used for double-buffer mode) 0x24000040

	DMA1_Stream3->CR |= DMA_SxCR_EN;				// Enable DMA and wait
	wait_loop_index = (SystemCoreClock / (100000UL * 2UL));
	while (wait_loop_index != 0UL) {
		wait_loop_index--;
	}

	ADC3->CR |= ADC_CR_ADSTART;						// Start ADC
}


void InitI2S()
{
	RCC->APB1LENR |= RCC_APB1LENR_SPI2EN;			//	Enable SPI clocks

	GpioPin::Init(GPIOB, 12, GpioPin::Type::AlternateFunction, 5);		// PB12: I2S2_WS [alternate function AF5]
	GpioPin::Init(GPIOB, 13, GpioPin::Type::AlternateFunction, 5);		// PB13: I2S2_CK [alternate function AF5]
	GpioPin::Init(GPIOB, 15, GpioPin::Type::AlternateFunction, 5);		// PB15: I2S2_SDO [alternate function AF5]

	// Configure SPI (Shown as SPI2->CGFR in SFR)
	SPI2->I2SCFGR |= SPI_I2SCFGR_I2SMOD;			// I2S Mode
	SPI2->I2SCFGR |= SPI_I2SCFGR_I2SCFG_1;			// I2S configuration mode: 00=Slave transmit; 01=Slave receive; 10=Master transmit*; 11=Master receive

	SPI2->I2SCFGR |= SPI_I2SCFGR_DATLEN_1;			// Data Length 00=16-bit; 01=24-bit; *10=32-bit
	SPI2->I2SCFGR |= SPI_I2SCFGR_CHLEN;				// Channel Length = 32bits

	SPI2->CFG1 |= SPI_CFG1_UDRCFG_1;				// In the event of underrun resend last transmitted data frame
	SPI2->CFG1 |= 0x1f << SPI_CFG1_DSIZE_Pos;		// Data size to 32 bits (FIFO holds 16 bytes = 4 x 32 bit words)
	SPI2->CFG1 |= 1 << SPI_CFG1_FTHLV_Pos;			// FIFO threshold level. 0001: 2-data; 0010: 3 data; 0011: 4 data

	/* I2S Clock
		PLL2: ((8MHz / 5) * 192 / 5) = 61.44 MHz
		I2S Clock: 61.44 MHz / (64  * ((2 * 10) + 0)) = 48 KHz
	*/

	RCC->D2CCIP1R |= RCC_D2CCIP1R_SPI123SEL_0;		// 001: pll2_p_ck clock selected as SPI/I2S1,2 and 3 kernel clock
	SPI2->I2SCFGR |= (10 << SPI_I2SCFGR_I2SDIV_Pos);	// Set I2SDIV to 10

	SPI2->CR1 |= SPI_CR1_SPE;						// Enable I2S

	SPI2->TXDR = (int32_t)0;						// Preload the FIFO
	SPI2->TXDR = (int32_t)0;
	SPI2->TXDR = (int32_t)0;
	SPI2->TXDR = (int32_t)0;

	SPI2->IER |= (SPI_IER_TXPIE | SPI_IER_UDRIE);	// Enable interrupt when FIFO has free slot or underrun occurs
	NVIC_SetPriority(SPI2_IRQn, 2);					// Lower is higher priority
	NVIC_EnableIRQ(SPI2_IRQn);


	SPI2->CR1 |= SPI_CR1_CSTART;					// Start I2S
}


void InitCordic()
{
	RCC->AHB2ENR |= RCC_AHB2ENR_CORDICEN;
}


void InitPWMTimer()
{
	// PD13: TIM4_CH2 LED_Filter_B
	// PD15: TIM4_CH4 LED_Filter_A
	RCC->APB1LENR |= RCC_APB1LENR_TIM4EN;

	GpioPin::Init(GPIOD, 13, GpioPin::Type::AlternateFunction, 2);		// Enable channel 1 PWM output pin on PE2
	GpioPin::Init(GPIOD, 15, GpioPin::Type::AlternateFunction, 2);		// Enable channel 2 PWM output pin on PE3

	TIM4->CCMR1 |= TIM_CCMR1_OC2PE;					// Output compare 2 preload enable
	TIM4->CCMR2 |= TIM_CCMR2_OC4PE;					// Output compare 4 preload enable

	TIM4->CCMR1 |= (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2);	// 0110: PWM mode 1 - In upcounting, channel 2 active if TIMx_CNT<TIMx_CCR2
	TIM4->CCMR2 |= (TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2);	// 0110: PWM mode 1 - In upcounting, channel 3 active if TIMx_CNT<TIMx_CCR3

	TIM4->CCR2 = 0;
	TIM4->CCR4 = 0;

	// Timing calculations: Clock = 500MHz / (PSC + 1) = 31m counts per second
	// ARR = number of counts per PWM tick = 4095
	// 31m / ARR ~= 7.6kHz of PWM square wave with 4095 levels of output

	TIM4->ARR = 4095;								// Total number of PWM ticks
	TIM4->PSC = 15;									// Prescaler
	TIM4->CR1 |= TIM_CR1_ARPE;						// 1: TIMx_ARR register is buffered
	TIM4->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC4E);	// Capture mode enabled / OC1 signal is output on the corresponding output pin
	TIM4->EGR |= TIM_EGR_UG;						// 1: Re-initialize the counter and generates an update of the registers

	TIM4->CR1 |= TIM_CR1_CEN;						// Enable counter
}


void InitDebugTimer()
{
	// Configure timer to use in internal debug timing - uses APB1 timer clock which is Main Clock [500MHz]
	// Each tick is 4ns with PSC 12nS - full range is 786.42 uS
	RCC->APB1LENR |= RCC_APB1LENR_TIM3EN;
	TIM3->ARR = 65535;
	TIM3->PSC = 2;
}


void StartDebugTimer()
{
	TIM3->EGR |= TIM_EGR_UG;
	TIM3->CR1 |= TIM_CR1_CEN;
}


float StopDebugTimer()
{
	// Return time running in microseconds
	const uint32_t count = TIM3->CNT;
	TIM3->CR1 &= ~TIM_CR1_CEN;
	return 0.0012 * count;
}


void DelayMS(uint32_t ms)
{
	// Crude delay system
	const uint32_t now = SysTickVal;
	while (now + ms > SysTickVal) {};
}


void JumpToBootloader()
{
	*reinterpret_cast<unsigned long *>(0x00000000) = 0xDEADBEEF; 	// Use ITCM RAM for DFU flag as this is not cleared at restart

	__disable_irq();
	SCB_DisableDCache();

	// Not sure why but seem to need to write this value twice or gets lost - caching issue?
	*reinterpret_cast<unsigned long *>(0x00000000) = 0xDEADBEEF;

	__DSB();
	NVIC_SystemReset();

	while (1) {
		// Code should never reach this loop
	}
}


void Reboot()
{
	__disable_irq();
	__DSB();

	SysTick->CTRL = 0;							// Disable Systick timer

	// Disable all peripheral clocks
	RCC->APB1LENR = 0;
	RCC->APB4ENR = 0;
	RCC->AHB1ENR = 0;
	RCC->APB2ENR = 0;
	RCC->AHB3ENR = 0;
	RCC->AHB4ENR = 0;

	for (uint32_t i = 0; i < 5; ++i) {			// Clear Interrupt Enable Register & Interrupt Pending Register
		NVIC->ICER[i] = 0xFFFFFFFF;
		NVIC->ICPR[i] = 0xFFFFFFFF;
	}

	NVIC_SystemReset();
}



#define ITCMRAM
#ifdef ITCMRAM
void CopyToITCMRAM()
{
	/* Load functions into ITCM RAM; To use add the following GCC attribute to function:

	 void __attribute__((section(".itcm_text"))) myFunction()

	 * The following section needs to be added to the linker script:

  itcm_data = LOADADDR(.itcm_text);
  .itcm_text :
  {
    . = ALIGN(4);
    itcm_text_start = .;
    *(.itcm_text)
    *(.itcm_text*)
    . = ALIGN(4);
    itcm_text_end = .;
  } >ITCMRAM AT> FLASH


	 * */
	extern  unsigned char itcm_text_start;
	extern const unsigned char itcm_text_end;
	extern const unsigned char itcm_data;
	memcpy(&itcm_text_start, &itcm_data, (int) (&itcm_text_end - &itcm_text_start));
}
#endif
