#include "LedManager.h"
#include <cstring>

LedManager ledManager;
uint8_t __attribute__((section (".dma_buffer"))) LedManager::leds[ledCount];

void LedManager::Init()
{
	// LED Controller on I2C5
	RCC->APB1LENR |= RCC_APB1LENR_I2C5EN;

	GpioPin::Init(GPIOC, 10, GpioPin::Type::AlternateFunction, 4, GpioPin::DriveStrength::VeryHigh, GpioPin::OutputType::OpenDrain);		// PC10: I2C5 SDA [alternate function AF4]
	GpioPin::Init(GPIOC, 11, GpioPin::Type::AlternateFunction, 4, GpioPin::DriveStrength::VeryHigh, GpioPin::OutputType::OpenDrain);		// PC11: I2C5 SCK [alternate function AF4]

	// By default clock source is APB1 peripheral clock (100 MHz)
	// 0x9034b6
	I2C5->TIMINGR = (110 << I2C_TIMINGR_SCLL_Pos) |	// SCL low period (master mode)
					(110 << I2C_TIMINGR_SCLH_Pos) |	// SCL high period (master mode)
					(9 << I2C_TIMINGR_SCLDEL_Pos);	// Data setup time

	I2C5->TIMINGR = (182 << I2C_TIMINGR_SCLL_Pos) |	// SCL low period (master mode)
					(52 << I2C_TIMINGR_SCLH_Pos) |	// SCL high period (master mode)
					(9 << I2C_TIMINGR_SCLDEL_Pos);	// Data setup time


	I2C5->CR1 |= I2C_CR1_DNF;						// Apply the maximum digital noise filter
	I2C5->CR2 |= I2C_CR2_AUTOEND;					// 1: Automatic end mode: STOP condition automatically sent when NBYTES data transferred
	I2C5->CR1 |= I2C_CR1_PE;						// Peripheral enable

	// Reset and then set default values
	WriteRegister(0x01, 0x08);						// Reset (clear all errors)
	WriteRegister(0x3F, 0x00);						// Set PWM to zero for all LEDs
	DelayMS(2);
	WriteRegister(0x40, 0x30);						// Set Current for all LEDs

	// Configure DMA
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

	DMA1_Stream0->CR &= ~DMA_SxCR_MSIZE;			// Memory size 8 bit [00 = 8 bit; 01 = 16 bit; 10 = 32 bit]
	DMA1_Stream0->CR &= ~DMA_SxCR_PSIZE_0;			// Peripheral size 8 bit
	DMA1_Stream0->CR |= DMA_SxCR_DIR_0;				// data transfer direction: 00: peripheral-to-memory; 01: memory-to-peripheral; 10: memory-to-memory
	DMA1_Stream0->CR |= DMA_SxCR_PL_0;				// Priority: 00 = low; 01 = Medium; 10 = High; 11 = Very High
	DMA1_Stream0->CR |= DMA_SxCR_MINC;				// Memory in increment mode
	DMA1_Stream0->PAR = (uint32_t)(&(I2C5->TXDR));	// Configure the peripheral data register address

	DMAMUX1_Channel0->CCR |= 125; 					// DMA request MUX input 125 = i2c5_tx_dma (See p.676)
	DMAMUX1_ChannelStatus->CFR |= DMAMUX_CFR_CSOF0; // Clear synchronization overrun event flag

	memset(leds, 0, ledCount);
	leds[0] = 0x8A;			// 0A is PWM for led 1 and 0x80 is autoincrement
	leds[10] = 20;
	leds[20] = 30;

}


void LedManager::WriteRegister(uint8_t reg, uint8_t value)
{
	while ((I2C5->ISR & I2C_ISR_TXE) == 0);

	// I2C_CR2_RELOAD - determines if transfer is completed after NBYTES transferred
	I2C5->ICR |= (I2C_ICR_NACKCF | I2C_ICR_BERRCF);			// Clear NAK and bus errors
	I2C5->TXDR = reg;
	I2C5->CR2 = (i2cAddress << (I2C_CR2_SADD_Pos + 1)) |	// 7 bit addresses are SADD[7:1]
				(2 << I2C_CR2_NBYTES_Pos) |
				I2C_CR2_START;
	// | I2C_CR2_STOP

	while ((I2C5->ISR & I2C_ISR_TXIS) == 0);
	I2C5->TXDR = value;
}


uint8_t LedManager::ReadRegister(uint8_t reg)
{
	while ((I2C5->ISR & I2C_ISR_TXE) == 0);

	I2C5->ICR |= (I2C_ICR_NACKCF | I2C_ICR_BERRCF);			// Clear NAK and bus errors
	I2C5->TXDR = reg;
	I2C5->CR2 = (i2cAddress << (I2C_CR2_SADD_Pos + 1)) |	// 7 bit addresses are SADD[7:1]
				(1 << I2C_CR2_NBYTES_Pos) |
				I2C_CR2_START;

	while ((I2C5->ISR & I2C_ISR_TXE) == 0);

	I2C5->CR2 = (i2cAddress << (I2C_CR2_SADD_Pos + 1)) |
				(1 << I2C_CR2_NBYTES_Pos) |
				I2C_CR2_RD_WRN |
				I2C_CR2_START;

	while ((I2C5->ISR & I2C_ISR_RXNE) == 0);

	return I2C5->RXDR;
}


void LedManager::DMASend()
{
	while ((I2C5->ISR & I2C_ISR_TXE) == 0);

	I2C5->CR1 &= ~I2C_CR1_PE;						// Disable SPI
	I2C5->ICR |= (I2C_ICR_NACKCF | I2C_ICR_BERRCF);	// Clear NAK and bus errors

	DMA1_Stream0->CR &= ~DMA_SxCR_EN;				// Disable DMA
	DMA1->LIFCR |= DMA_LIFCR_CFEIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0;	// Clear DMA errors and transfer complete status flags

	DMA1_Stream0->M0AR = (uint32_t)leds;			// Configure the memory data register address
	DMA1_Stream0->NDTR = ledCount;					// Number of data items to transfer
	DMA1_Stream0->CR |= DMA_SxCR_EN;				// Enable DMA and wait

	I2C5->CR1 |= I2C_CR1_TXDMAEN;					// Tx DMA stream enable
	I2C5->CR1 |= I2C_CR1_PE;						// Enable I2C
	I2C5->CR2 = (i2cAddress << (I2C_CR2_SADD_Pos + 1)) |	// 7 bit addresses are SADD[7:1]
				(ledCount << I2C_CR2_NBYTES_Pos) |
				I2C_CR2_START;
}


bool LedManager::Ready()
{
	return (I2C5->ISR & I2C_ISR_TC);				// Use transfer complete flag to indicate ready for next transfer
}
