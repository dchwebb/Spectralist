#include "uartHandler.h"
#include "GpioPin.h"
#include "usb.h"

#if (USB_DEBUG)
UART uart;
#endif

void UART::Init() {
	// Debug UART pin: PC10 = USART3_TX; PC11 = USART3_RX
	RCC->APB1LENR |= RCC_APB1LENR_USART3EN;			// USART clock enable

	GpioPin::Init(GPIOC, 10, GpioPin::Type::AlternateFunction, 7);	// PC10 USART3_TX    AF7
	GpioPin::Init(GPIOC, 11, GpioPin::Type::AlternateFunction, 7);	// PC11 USART3_RX    AF7

	// Calculations depended on oversampling mode set in CR1 OVER8. Default = 0: Oversampling by 16
	USART3->BRR |= SystemCoreClock / 230400;		// clk / desired_baud
	USART3->CR1 &= ~USART_CR1_M;					// 0: 1 Start bit, 8 Data bits, n Stop bit; 1: 1 Start bit, 9 Data bits, n Stop bit
	USART3->CR1 |= USART_CR1_RE;					// Receive enable
	USART3->CR1 |= USART_CR1_TE;					// Transmitter enable

	// Set up interrupts
	USART3->CR1 |= USART_CR1_RXNEIE;
	NVIC_SetPriority(USART3_IRQn, 6);				// Lower is higher priority
	NVIC_EnableIRQ(USART3_IRQn);

	USART3->CR1 |= USART_CR1_UE;					// UART Enable
}


void UART::SendChar(char c) {
	while ((USART3->ISR & USART_ISR_TXE_TXFNF) == 0);
	USART3->TDR = c;
}

void UART::SendString(const char* s) {
	char c = s[0];
	uint8_t i = 0;
	while (c) {
		while ((USART3->ISR & USART_ISR_TXE_TXFNF) == 0);
		USART3->TDR = c;
		c = s[++i];
	}
}

void UART::SendString(const std::string& s) {
	for (char c : s) {
		while ((USART3->ISR & USART_ISR_TXE_TXFNF) == 0);
		USART3->TDR = c;
	}
}


void UART::ProcessCommand()
{
	if (!commandReady) {
		return;
	}
	std::string_view cmd {command};

#if (USB_DEBUG)
	if (cmd.compare("printdebug\n") == 0) {
		usb.OutputDebug();

	} else if (cmd.compare("debugon\n") == 0) {
		extern volatile bool debugStart;
		debugStart = true;
		SendString("Debug activated\r\n");
	} else {
		SendString("Unrecognised command\r\n");
	}
#endif
	commandReady = false;
}


extern "C" {

// USART Decoder
void USART3_IRQHandler() {
#if (USB_DEBUG)
	if (!uart.commandReady) {
		const uint32_t recData = USART3->RDR;					// Note that 32 bits must be read to clear the receive flag
		uart.command[uart.cmdPos] = (char)recData; 				// accessing RDR automatically resets the receive flag
		if (uart.command[uart.cmdPos] == 10) {
			uart.commandReady = true;
			uart.cmdPos = 0;
		} else {
			uart.cmdPos++;
		}
	}
#endif
}
}
