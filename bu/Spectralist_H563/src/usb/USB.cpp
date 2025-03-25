#pragma GCC push_options
#pragma GCC optimize ("O0")

#include "USB.h"

bool USBDebug = true;		// Used if outputting debug over USB
USBMain usb;

USB_PMA_t* USB_PMA = (USB_PMA_t*)USB_DRD_PMAADDR;		// To access PMA registers as array
USB_EPR_t* USB_EPR = (USB_EPR_t*)(&USBP->CHEP0R);		// To access Endpoint registers as array

extern "C" {
// To enable USB for printf commands (To print floats enable 'Use float with printf from newlib-nano' MCU Build Settings)
size_t _write(int handle, const unsigned char* buf, size_t bufSize)
{
	if (usb.devState == USBMain::DeviceState::Configured) {
		return usb.SendString(buf, bufSize);
	} else {
		++usb.stringErrors;
		return 0;
	}
}
}

inline void ClearRxInterrupt(const uint8_t ep)
{
	const uint16_t wRegVal = (USB_EPR[ep].EPR & USB_CHEP_REG_MASK) & ~USB_EP_VTRX;
	USB_EPR[ep].EPR = wRegVal | USB_EP_VTTX;
}

inline void ClearTxInterrupt(const uint8_t ep)
{
	const uint16_t wRegVal = (USB_EPR[ep].EPR & USB_CHEP_REG_MASK) & ~USB_EP_VTTX;
	USB_EPR[ep].EPR = wRegVal | USB_EP_VTRX;
}


inline void SetTxStatus(const uint8_t ep, const uint16_t status)		// Set endpoint transmit status - have to use XOR to toggle bits
{
	const uint16_t wRegVal = (USB_EPR[ep].EPR & USB_CHEP_TX_DTOGMASK) ^ status;
	USB_EPR[ep].EPR = wRegVal | USB_EP_VTRX | USB_EP_VTTX;
}


inline void SetRxStatus(const uint8_t ep, const uint16_t status)		// Set endpoint receive status - have to use XOR to toggle bits
{
	const uint16_t wRegVal = (USB_EPR[ep].EPR & USB_CHEP_RX_DTOGMASK) ^ status;
	USB_EPR[ep].EPR = wRegVal | USB_EP_VTRX | USB_EP_VTTX;
}


void USBMain::ReadPMA(const uint32_t pma, USBHandler* handler)
{
	volatile uint32_t* pmaBuff = reinterpret_cast<volatile uint32_t*>(USB_DRD_PMAADDR + pma);		// Eg 0x40006018

	for (uint32_t i = 0; i < (handler->outBuffCount + 3) / 4; i++) {
		reinterpret_cast<volatile uint32_t*>(handler->outBuff)[i] = *pmaBuff++;				// pma buffer can only be read in 16 bit words
	}

#if (USB_DEBUG)
	usbDebug[usbDebugNo].PacketSize = handler->outBuffCount;
	usbDebug[usbDebugNo].xferBuff0 = ((uint32_t*)handler->outBuff)[0];
	usbDebug[usbDebugNo].xferBuff1 = ((uint32_t*)handler->outBuff)[1];
#endif
}


void USBMain::WritePMA(const uint32_t pma, const uint32_t bytes, USBHandler* handler)
{
	volatile uint32_t* pmaBuff = reinterpret_cast<volatile uint32_t*>(USB_DRD_PMAADDR + pma);

	for (uint32_t i = 0; i < (bytes + 3) / 4; i++) {
		pmaBuff[i] = reinterpret_cast<const uint32_t*>(handler->inBuff)[i];
	}
}


void USBMain::ProcessSetupPacket()
{
	req.loadData((uint8_t*)classByEP[0]->outBuff);		// Parse the setup request into the req object

#if (USB_DEBUG)
	usbDebug[usbDebugNo].Request = req;
#endif
	// Previously USBD_StdDevReq
	if ((req.RequestType & recipientMask) == RequestRecipientDevice && (req.RequestType & requestTypeMask) == RequestTypeStandard) {
		switch (static_cast<Request>(req.Request)) {
		case Request::GetDescriptor:
			GetDescriptor();
			break;

		case Request::SetAddress:
			devAddress = static_cast<uint8_t>(req.Value) & 0x7F;			// Address address is set on the next interrupt - hold in temp storage

			devState = DeviceState::Addressed;
			EPStartXfer(Direction::in, 0, 0);

			break;

		case Request::SetConfiguration:
			if (devState == DeviceState::Addressed) {
				devState = DeviceState::Configured;

				for (auto c : classByEP) {
					c->ActivateEP();
				}

				EPStartXfer(Direction::in, 0, 0);
			}
			break;

		default:
			SetTxStatus(0, USB_EP_TX_STALL);
			break;
		}

	// Previously USBD_StdItfReq
	} else if ((req.RequestType & recipientMask) == RequestRecipientInterface && (req.RequestType & requestTypeMask) == RequestTypeClass) {

		// req.Index holds interface - call the appropriate handler's setup
		if (req.Length > 0) {
			classesByInterface[req.Index]->ClassSetup(req);
		} else {
			EPStartXfer(Direction::in, 0, 0);
		}

	} else {
		SetTxStatus(0, USB_EP_TX_STALL);
	}
}


// EPStartXfer setup and starts a transfer over an EP
void USBMain::EPStartXfer(const Direction direction, const uint8_t endpoint, uint32_t len)
{
	const uint8_t epIndex = (endpoint & epAddrMask);

	if (direction == Direction::in) {						// IN endpoint
		if (len > ep_maxPacket) {
			len = ep_maxPacket;
		}

		WritePMA(USB_PMA[epIndex].GetTXAddr(), len, classByEP[epIndex]);
		USB_PMA[epIndex].SetTXCount(len);

#if (USB_DEBUG)
				usbDebug[usbDebugNo].PacketSize = len;
				if (len > 0) {
					usbDebug[usbDebugNo].xferBuff0 = ((uint32_t*)classByEP[epIndex]->inBuff)[0];
					usbDebug[usbDebugNo].xferBuff1 = ((uint32_t*)classByEP[epIndex]->inBuff)[1];
				}
#endif

		SetTxStatus(epIndex, USB_EP_TX_VALID);
	} else {												// OUT endpoint
		SetRxStatus(0, USB_EP_RX_VALID);
	}
}


void USBMain::InterruptHandler()							// Originally in Drivers\STM32F4xx_HAL_Driver\Src\stm32f4xx_hal_pcd.c
{
	// Handle spurious interrupt
	USBP->ISTR &= ~(USB_ISTR_SOF | USB_ISTR_ESOF);
	if ((USBP->ISTR) == 0) {
		return;
	}


	/////////// 	8000 		USB_ISTR_CTR: Correct Transfer
	while (ReadInterrupts(USB_ISTR_CTR)) {					// Originally PCD_EP_ISR_Handler
		const uint8_t epIndex = USBP->ISTR & USB_ISTR_IDN;	// Extract highest priority endpoint number

#if (USB_DEBUG)
		usbDebug[usbDebugNo].endpoint = epIndex;
#endif

		if (epIndex == 0) {
			if ((USBP->ISTR & USB_ISTR_DIR) == 0) {			// DIR = 0: Direction IN
				ClearTxInterrupt(0);

				const uint16_t txBytes = USB_PMA[0].GetTXCount();
				classByEP[epIndex]->inBuff += txBytes;

				if (classByEP[epIndex]->inBuffRem > ep_maxPacket) {
					classByEP[epIndex]->inBuffRem -= ep_maxPacket;
					EPStartXfer(Direction::in, 0, classByEP[epIndex]->inBuffRem);
					EPStartXfer(Direction::out, 0, 0);
				} else {
					// FIXME if (rem_length ==  maxpacket) etc - where non zero size packet and last packet is a multiple of max packet size
					SetTxStatus(0, USB_EP_TX_STALL);
					EPStartXfer(Direction::out, 0, 0);
				}

				if (devAddress > 0 && txBytes == 0) {
					USBP->DADDR = (devAddress | USB_DADDR_EF);
					devAddress = 0;
				}

			} else {										// DIR = 1: Setup or OUT interrupt
				ClearRxInterrupt(0);

				if ((USBP->CHEP0R & USB_EP_SETUP) != 0) {
					classByEP[0]->outBuffCount = USB_PMA[0].GetRXCount();
					ReadPMA(USB_PMA[0].GetRXAddr(), classByEP[0]);	// Read setup data into  receive buffer
					ProcessSetupPacket();					// Parse setup packet into request, locate data (eg descriptor) and populate TX buffer

				} else {
					classByEP[0]->outBuffCount = USB_PMA[0].GetRXCount();
					if (classByEP[0]->outBuffCount != 0) {
						ReadPMA(USB_PMA[0].GetRXAddr(), classByEP[0]);

						if (devState == DeviceState::Configured && classPendingData) {
							if ((req.RequestType & requestTypeMask) == RequestTypeClass) {
								// Previous OUT interrupt contains instruction (eg host sending CDC LineCoding); next command sends data (Eg LineCoding data)
								classesByInterface[req.Index]->ClassSetupData(req, (uint8_t*)ep0.outBuff);
							}
							classPendingData = false;
							EPStartXfer(Direction::in, 0, 0);
						}
					}
					SetRxStatus(0, USB_EP_RX_VALID);
				}
			}

		} else {
			// Non zero endpoint
			if ((USB_EPR[epIndex].EPR & USB_EP_VTRX) != 0) {
				ClearRxInterrupt(epIndex);
				
				classByEP[epIndex]->outBuffCount = USB_PMA[epIndex].GetRXCount();
				if (classByEP[epIndex]->outBuffCount != 0) {
					ReadPMA(USB_PMA[epIndex].GetRXAddr(), classByEP[epIndex]);
				}
				SetRxStatus(epIndex, USB_EP_RX_VALID);


				classByEP[epIndex]->DataOut();
			}

			if ((USB_EPR[epIndex].EPR & USB_EP_VTTX) != 0) {

				ClearTxInterrupt(epIndex);

				const uint16_t txBytes = USB_PMA[epIndex].GetTXCount();
				if (classByEP[epIndex]->inBuffSize >= txBytes) {					// Transmitting data larger than buffer size
					classByEP[epIndex]->inBuffSize -= txBytes;
					classByEP[epIndex]->inBuff += txBytes;

					// Send IN packet if data to be sent, or size of last block in sequence is exactly size of maximum packet
					if (classByEP[epIndex]->inBuffSize > 0 || (txBytes == ep_maxPacket && classByEP[epIndex]->inBuffSize == 0)) {
						EPStartXfer(Direction::in, epIndex, classByEP[epIndex]->inBuffSize);
					}
				}
				if (classByEP[epIndex]->inBuffSize == 0) {
					transmitting = false;
					classByEP[epIndex]->DataIn();
				}

			}

		}
	}


	/////////// 	1000 		USB_ISTR_WKUP: Wake Up
	if (ReadInterrupts(USB_ISTR_WKUP)) {
		USBP->CNTR &= ~USB_CNTR_SUSPEN;
		//USBP->CNTR &= ~USB_CNTR_LPMODE;		// FIXME
		USBP->ISTR &= ~USB_ISTR_WKUP;
	}

	/////////// 	800 		SUSP: Suspend Interrupt
	if (ReadInterrupts(USB_ISTR_SUSP)) {
		USBP->CNTR |= USB_CNTR_SUSPEN;
		USBP->ISTR &= ~USB_ISTR_SUSP;
		//USBP->CNTR |= USB_CNTR_LPMODE;		// FIXME
		devState = DeviceState::Suspended;
	}

	/////////// 	400 		RESET: Reset Interrupt
	if (ReadInterrupts(USB_ISTR_RESET))	{
		USBP->ISTR &= ~USB_ISTR_RESET;

		pmaAddress = pmaStartAddr;						// Reset PMA allocation start address
		ActivateEndpoint(0, Direction::out, Control);
		ActivateEndpoint(0, Direction::in,  Control);

		USBP->DADDR = USB_DADDR_EF;						// Enable endpoint and set address to 0
	}

	/////////// 	100 		USB_ISTR_ESOF: Expected Start of frame
	if (ReadInterrupts(USB_ISTR_ESOF)) {
		USBP->ISTR &= ~USB_ISTR_ESOF;
	}

	/////////// 	2000 		ERR: Error Interrupt
	if (ReadInterrupts(USB_ISTR_ERR)) {
		USBP->ISTR &= ~USB_ISTR_ERR;
	}
}


void USBMain::InitUSB()
{
	RCC->APB2ENR |= RCC_APB2ENR_USBEN;					// USB2OTG (OTG_HS2) Peripheral Clocks Enable
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;				// Enable GPIO Clock

	RCC->CR |= RCC_CR_HSI48ON;							// Enable USB clock
	while ((RCC->CR & RCC_CR_HSI48RDY) == 0);			// Wait till ready

	RCC->CCIPR4 |= RCC_CCIPR4_USBSEL;					// USB clock source: 00: no clock; 01: pll1_q_ck; 10: pll3_q_ck; *11: hsi48_ker_ck

	// MODER 00: Input mode, 01: General purpose output mode, 10: Alternate function mode, 11: Analog mode (reset state)
	GPIOA->MODER &= ~(GPIO_MODER_MODE11_0 | GPIO_MODER_MODE12_0);
	GPIOA->AFR[1] |= (10 << GPIO_AFRH_AFSEL11_Pos) | (10 << GPIO_AFRH_AFSEL12_Pos);

	NVIC_SetPriority(USB_DRD_FS_IRQn, 3);
	NVIC_EnableIRQ(USB_DRD_FS_IRQn);

	USBP->CNTR &= ~USB_CNTR_HOST;						// Set mode to device
	USBP->CNTR |= USB_CNTR_USBRST;						// Force USB Reset
	USBP->CNTR &= ~USB_CNTR_USBRST;						// Release reset
	USBP->ISTR = 0;										// Clear pending interrupts
	USBP->CNTR = USB_CNTR_CTRM  | USB_CNTR_WKUPM | USB_CNTR_SUSPM | USB_CNTR_ERRM | USB_CNTR_RESETM | USB_CNTR_L1REQM;

	USBP->BCDR |= USB_BCDR_DPPU;						// Connect internal PU resistor on USB DP line

#if (USB_DEBUG)
	uart.Init();
#endif
}


void USBMain::ActivateEndpoint(uint8_t endpoint, const Direction direction, EndPointType endpointType)
{
	endpoint = endpoint & epAddrMask;
	uint16_t epType = 0;
	switch (endpointType) {
		case Control:		epType = USB_EP_CONTROL;		break;
		case Isochronous:	epType = USB_EP_ISOCHRONOUS;	break;
		case Bulk:			epType = USB_EP_BULK;			break;
		case Interrupt:		epType = USB_EP_INTERRUPT;		break;
	}

	// Set the address (EA=endpoint) and type (EP_TYPE=ep_type)
	USB_EPR[endpoint].EPR = (USB_EPR[endpoint].EPR & USB_EP_T_MASK) | (endpoint | epType | USB_EP_VTRX | USB_EP_VTTX);

	if (direction == Direction::in) {
		USB_PMA[endpoint].SetTXAddr(pmaAddress);			// Offset of PMA used for EP TX

		// Clear tx data toggle (data packets must alternate 1 and 0 in the data field)
		if ((USB_EPR[endpoint].EPR & USB_EP_DTOG_TX) != 0) {
			const uint16_t wEPVal = USB_EPR[endpoint].EPR & USB_CHEP_REG_MASK;
			USB_EPR[endpoint].EPR = wEPVal | USB_EP_VTRX | USB_EP_VTTX | USB_EP_DTOG_TX;
		}

		SetTxStatus(endpoint, USB_EP_TX_NAK);

	} else {
		USB_PMA[endpoint].SetRXAddr(pmaAddress);			// Offset of PMA used for EP RX
		USB_PMA[endpoint].SetRXBlkSize(1);					// configure block size = 1 (32 Bytes)
		USB_PMA[endpoint].SetRXBlocks(1);					// number of blocks = 2 (64 bytes)

		// Clear rx data toggle
		if ((USB_EPR[endpoint].EPR & USB_EP_DTOG_RX) != 0) {
			const uint16_t wEPVal = USB_EPR[endpoint].EPR & USB_CHEP_REG_MASK;
			USB_EPR[endpoint].EPR = wEPVal | USB_EP_VTRX | USB_EP_VTTX | USB_EP_DTOG_RX;
		}

		SetRxStatus(endpoint, USB_EP_RX_VALID);
	}

	// Increment PMA address in 64 byte chunks
	pmaAddress += ep_maxPacket;
}


// procedure to allow classes to pass configuration data back via endpoint 0 (eg CDC line setup, MSC MaxLUN etc)
void USBMain::EP0In(const uint8_t* buff, const uint32_t size)
{
	ep0.inBuff = buff;
	ep0.inBuffRem = size;
	ep0.inBuffSize = std::min(size, static_cast<uint32_t>(req.Length));
	EPStartXfer(Direction::in, 0, ep0.inBuffSize);		// sends blank request back

#if (USB_DEBUG)
	usbDebug[usbDebugNo].PacketSize = ep0.inBuffSize;
	usbDebug[usbDebugNo].xferBuff0 = ((uint32_t*)ep0.inBuff)[0];
	usbDebug[usbDebugNo].xferBuff1 = ((uint32_t*)ep0.inBuff)[1];
#endif
}


void USBMain::GetDescriptor()
{
	uint32_t strSize;

	switch (static_cast<Descriptor>(req.Value >> 8))	{
	case DeviceDescriptor:
		return EP0In(USBD_FS_DeviceDesc, sizeof(USBD_FS_DeviceDesc));
		break;

	case ConfigurationDescriptor:
		return EP0In(configDescriptor, MakeConfigDescriptor());		// Construct config descriptor from individual classes
		break;

	case BosDescriptor:
		return EP0In(USBD_FS_BOSDesc, sizeof(USBD_FS_BOSDesc));
		break;

	case StringDescriptor:

		switch ((uint8_t)(req.Value)) {
		case StringIndex::LangId:
			return EP0In(USBD_LangIDDesc, sizeof(USBD_LangIDDesc));
			break;

		case StringIndex::Manufacturer:
			strSize = StringToUnicode(manufacturerString, stringDescr);
			return EP0In(stringDescr, strSize);
			break;

		case StringIndex::Product:
			strSize = StringToUnicode(productString, stringDescr);
			return EP0In(stringDescr, strSize);
			break;

		case StringIndex::Serial:
			SerialToUnicode();
			return EP0In(stringDescr, stringDescr[0]);				// length is 24 bytes (x2 for unicode padding) + 2 for header
			break;

	    case StringIndex::CommunicationClass:
	    	strSize = StringToUnicode(cdcString, stringDescr);
			return EP0In(stringDescr, strSize);
			break;

		case StringIndex::AudioClass:
			strSize = StringToUnicode(midiString, stringDescr);
	    	return EP0In(stringDescr, strSize);
	    	break;

		default:
			SetTxStatus(0, USB_EP_TX_STALL);
			return;
		}
		break;

	default:
		SetTxStatus(0, USB_EP_TX_STALL);
		return;
	}

	if (req.Length == 0) {
		EPStartXfer(Direction::in, 0, 0);
	}
}


uint32_t USBMain::MakeConfigDescriptor()
{
	// Construct the configuration descriptor from the various class descriptors with header
	static constexpr uint8_t descrHeaderSize = 9;
	uint32_t descPos = descrHeaderSize;
	for (auto c : classByEP) {
		if (c != nullptr) {
			const uint8_t* descBuff = nullptr;
			const uint32_t descSize = c->GetInterfaceDescriptor(&descBuff);
			memcpy(&configDescriptor[descPos], descBuff, descSize);
			descPos += descSize;
		}
	}

	// Insert config descriptor header
	const uint8_t descriptorHeader[] = {
		descrHeaderSize,					// bLength: Configuration Descriptor size
		ConfigurationDescriptor,			// bDescriptorType: Configuration
		LOBYTE(descPos),					// wTotalLength
		HIBYTE(descPos),
		interfaceCount,						// bNumInterfaces
		0x01,								// bConfigurationValue: Configuration value
		0x04,								// iConfiguration: Index of string descriptor describing the configuration
		0x80 | (selfPowered << 6),			// bmAttributes: self powered
		0x32,								// MaxPower 0 mA
	};
	memcpy(&configDescriptor[0], descriptorHeader, descrHeaderSize);

	return descPos;
}


uint32_t USBMain::StringToUnicode(const std::string_view desc, uint8_t *unicode)
{
	uint32_t idx = 2;
	for (auto c: desc) {
		unicode[idx++] = c;
		unicode[idx++] = 0;
	}
	unicode[0] = idx;
	unicode[1] = StringDescriptor;

	return idx;
}


void USBMain::SerialToUnicode()
{
	const uint32_t* uidAddr = (uint32_t*)UID_BASE;			// Location in memory that holds 96 bit Unique device ID register

	char uidBuff[usbSerialNoSize + 1];
	snprintf(uidBuff, usbSerialNoSize + 1, "%08lx%08lx%08lx", uidAddr[0], uidAddr[1], uidAddr[2]);

	stringDescr[0] = usbSerialNoSize * 2 + 2;				// length is 24 bytes (x2 for unicode padding) + 2 for header
	stringDescr[1] = StringDescriptor;
	for (uint8_t i = 0; i < usbSerialNoSize; ++i) {
		stringDescr[i * 2 + 2] = uidBuff[i];
	}
}


bool USBMain::ReadInterrupts(const uint32_t interrupt)
{
#if (USB_DEBUG)
	if ((USBP->ISTR & interrupt) == interrupt && usbDebugEvent < USB_DEBUG_COUNT) {
		usbDebugNo = usbDebugEvent % USB_DEBUG_COUNT;
		usbDebug[usbDebugNo].eventNo = usbDebugEvent;
		usbDebug[usbDebugNo].Interrupt = USBP->ISTR;
		usbDebugEvent++;
	}
#endif

	return (USBP->ISTR & interrupt) == interrupt;
}


size_t USBMain::SendData(const uint8_t* data, uint16_t len, uint8_t endpoint)
{
	if (devState == DeviceState::Configured && !transmitting) {
		transmitting = true;
		classByEP[endpoint & epAddrMask]->inBuff = (uint8_t*)data;
		classByEP[endpoint & epAddrMask]->inBuffSize = len;
		EPStartXfer(Direction::in, endpoint, len);
		return len;
	} else {
		++stringErrors;
		return 0;
	}
}


void USBMain::SendString(const char* s)
{
	uint16_t counter = 0;
	while (transmitting && counter < 10000) {
		++stringErrors;
		++counter;
	}
	SendData((uint8_t*)s, strlen(s), CDC_In);
}


void USBMain::SendString(const std::string& s)
{
	SendString(s.c_str());
}


size_t USBMain::SendString(const unsigned char* s, size_t len)
{
	uint16_t counter = 0;
	while (transmitting && counter < 10000) {
		++stringErrors;
		++counter;
	}
	return SendData((uint8_t*)s, len, CDC_In);
}

#if (USB_DEBUG)

std::string IntToString(const int32_t& v) {
	return std::to_string(v);
}

std::string HexToString(const uint32_t& v, const bool& spaces) {
	char buf[20];
	if (spaces) {
		if (v != 0) {
			uint8_t* bytes = (uint8_t*)&v;
			sprintf(buf, "%02X%02X%02X%02X", bytes[0], bytes[1], bytes[2], bytes[3]);
		} else {
			sprintf(buf, " ");
		}
	} else {
		sprintf(buf, "%X", (unsigned int)v);
	}
	return std::string(buf);

}

std::string HexByte(const uint16_t& v) {
	char buf[50];
	sprintf(buf, "%X", v);
	return std::string(buf);

}

void USBMain::OutputDebug()
{
	USBDebug = false;

	uart.SendString("Event,Interrupt,Name,Desc,Endpoint,mRequest,Request,Value,Index,Length,PacketSize,XferBuff,\n");
	uint16_t evNo = usbDebugEvent % USB_DEBUG_COUNT;
	std::string interrupt, subtype;

	for (int i = 0; i < USB_DEBUG_COUNT; ++i) {
		if ((usbDebug[evNo].Interrupt & USB_ISTR_CTR) == USB_ISTR_CTR) {
			if ((usbDebug[evNo].Interrupt & USB_ISTR_DIR) == USB_ISTR_DIR) {
				interrupt = "CTR_OUT";
				if (usbDebug[evNo].Request.Request == 6) {
					switch (static_cast<Descriptor>(usbDebug[evNo].Request.Value >> 8))	{
					case DeviceDescriptor:
						subtype = "Get Device Descriptor";
						break;
					case ConfigurationDescriptor:
						subtype = "Get Configuration Descriptor";
						break;
					case BosDescriptor:
						subtype = "Get BOS Descriptor";
						break;

					case StringDescriptor:

						switch ((uint8_t)(usbDebug[evNo].Request.Value & 0xFF)) {
						case StringIndex::LangId:				// 300
							subtype = "Get Lang Str Descriptor";
							break;
						case StringIndex::Manufacturer:					// 301
							subtype = "Get Manufacturor Str Descriptor";
							break;
						case StringIndex::Product:				// 302
							subtype = "Get Product Str Descriptor";
							break;
						case StringIndex::Serial:				// 303
							subtype = "Get Serial Str Descriptor";
							break;
					    case StringIndex::CommunicationClass:					// 304
							subtype = "Get CDC Str Descriptor";
							break;
						}
						break;
					default:
						subtype = "Get Descriptor";
					}
				} else if (usbDebug[evNo].Request.Request == 5) {
					subtype = "Set Address to " + std::to_string(usbDebug[evNo].Request.Value);
				} else if (usbDebug[evNo].Request.Request == 9) {
					subtype = "SET_CONFIGURATION";
				} else if ((usbDebug[evNo].Request.RequestType & requestTypeMask) == RequestTypeClass) {
					switch (usbDebug[evNo].Request.Request) {
					case 0x20:
						subtype = "CDC: Set Line Coding";
						break;
					case 0x21:
						subtype = "CDC: Get Line Coding";
						break;
					case 0x22:
						subtype = "CDC: Set Control Line State";
						break;
					}
				} else {
					subtype = "";
				}
			} else {
				interrupt = "CTR_IN";
				subtype = "";
			}
		}

		if ((usbDebug[evNo].Interrupt & USB_ISTR_SUSP) == USB_ISTR_SUSP) {
			interrupt = "SUSP";
		}

		if ((usbDebug[evNo].Interrupt & USB_ISTR_WKUP) == USB_ISTR_WKUP) {
			interrupt = "WKUP";
		}

		if ((usbDebug[evNo].Interrupt & USB_ISTR_RESET) == USB_ISTR_RESET) {
			interrupt = "RESET";
		}


		if (usbDebug[evNo].Interrupt != 0) {
			uart.SendString(std::to_string(usbDebug[evNo].eventNo) + ","
					+ HexToString(usbDebug[evNo].Interrupt, false) + ","
					+ interrupt + "," + subtype + ","
					+ std::to_string(usbDebug[evNo].endpoint) + ","
					+ HexByte(usbDebug[evNo].Request.RequestType) + ","
					+ HexByte(usbDebug[evNo].Request.Request) + ","
					+ HexByte(usbDebug[evNo].Request.Value) + ","
					+ HexByte(usbDebug[evNo].Request.Index) + ","
					+ HexByte(usbDebug[evNo].Request.Length) + ","
					+ HexByte(usbDebug[evNo].PacketSize) + ","
					+ HexToString(usbDebug[evNo].xferBuff0, true) + " "
					+ HexToString(usbDebug[evNo].xferBuff1, true) + "\n");

		}
		evNo = (evNo + 1) % USB_DEBUG_COUNT;
	}
}


/* startup sequence:

Event	Int	Name	Desc							ep	mReq	Req	Val	Idx	Len	PacketSize	XferBuff
0	400		RESET									0	0		0	0	0	0	0
1	800		SUSP									0	0		0	0	0	0	0
2	1000	WKUP									0	0		0	0	0	0	0
3	400		RESET									0	0		0	0	0	0	0
4	8010	CTR_OUT	Get Device Descriptor			0	80		6	100	0	40	12			640C0020 EF020140
5	8000	CTR_IN									0	0		0	0	0	0	0
6	8010	CTR_OUT									0	0		0	0	0	0	0
7	8010	CTR_OUT	Set Address to 29				0	0		5	1D	0	0	0			00051D00
8	8000	CTR_IN									0	0		0	0	0	0	0
9	8010	CTR_OUT	Get Device Descriptor			0	80		6	100	0	12	12			640C0020 EF020140
10	8000	CTR_IN									0	0		0	0	0	0	0
11	8010	CTR_OUT									0	0		0	0	0	0	0
12	8010	CTR_OUT	Get Configuration Descriptor	0	80		6	200	0	FF	98			650B0020 040104C0
13	8000	CTR_IN									0	0		0	0	0	0	40			00000005 25010101
14	8000	CTR_IN									0	0		0	0	0	0	18			10090403 00020A00
15	8000	CTR_IN									0	0		0	0	0	0	0
16	8010	CTR_OUT									0	0		0	0	0	0	0
17	8010	CTR_OUT	Get BOS Descriptor				0	80		6	F00	0	FF	C			760C0020 01071002
18	8000	CTR_IN									0	0		0	0	0	0	0
19	8010	CTR_OUT									0	0		0	0	0	0	0
20	8010	CTR_OUT	Get Serial Str Descriptor		0	80		6	303	409	FF	32			E50A0020 6F007500
21	8000	CTR_IN									0	0		0	0	0	0	0
22	8010	CTR_OUT									0	0		0	0	0	0	0
23	8010	CTR_OUT	Get Lang Str Descriptor			0	80		6	300	0	FF	4			820C0020 17001800
24	8000	CTR_IN									0	0		0	0	0	0	0
25	8010	CTR_OUT									0	0		0	0	0	0	0
26	8010	CTR_OUT	Get Product Str Descriptor		0	80		6	302	409	FF	22			E50A0020 6F007500
27	8000	CTR_IN									0	0		0	0	0	0	0
28	8010	CTR_OUT									0	0		0	0	0	0	0
29	8010	CTR_OUT	Get Descriptor					0	80		6	600	0	A	8			80060006 00000A00
30	8010	CTR_OUT	SET_CONFIGURATION				0	0		9	1	0	0	0			90100
31	8000	CTR_IN									0	0		0	0	0	0	0
32	8010	CTR_OUT									0	80		6	307	409	4	4			E50A0020 6F007500
33	8000	CTR_IN									0	0		0	0	0	0	0
34	8010	CTR_OUT									0	0		0	0	0	0	0
35	8010	CTR_OUT									0	80		6	307	409	2C	2C			E50A0020 6F007500
36	8000	CTR_IN									0	0		0	0	0	0	0
37	8010	CTR_OUT									0	0		0	0	0	0	0
38	8010	CTR_OUT	Get CDC Str Descriptor			0	80		6	306	409	4	4			E50A0020 6F007500
39	8000	CTR_IN									0	0		0	0	0	0	0
40	8010	CTR_OUT									0	0		0	0	0	0	0
41	8010	CTR_OUT	Get CDC Str Descriptor			0	80		6	306	409	2A	2A			E50A0020 6F007500
42	8000	CTR_IN									0	0		0	0	0	0	0
43	8010	CTR_OUT									0	0		0	0	0	0	0
44	8010	CTR_OUT									0	80		6	307	409	4	4			E50A0020 6F007500
45	8000	CTR_IN									0	0		0	0	0	0	0
46	8010	CTR_OUT									0	0		0	0	0	0	0
47	8010	CTR_OUT									0	80		6	307	409	2C	2C			E50A0020 6F007500
48	8000	CTR_IN									0	0		0	0	0	0	0
49	8010	CTR_OUT									0	0		0	0	0	0	0
50	8010	CTR_OUT	Get Lang Str Descriptor			0	80		6	300	0	FF	4			820C0020 32003300
51	8000	CTR_IN									0	0		0	0	0	0	0
52	8010	CTR_OUT									0	0		0	0	0	0	0
53	8010	CTR_OUT	Get Lang Str Descriptor			0	80		6	300	0	FF	4			820C0020 35003600
54	8000	CTR_IN									0	0		0	0	0	0	0
55	8010	CTR_OUT									0	0		0	0	0	0	0
56	8010	CTR_OUT	Get Manufacturor Str Descr		0	80		6	301	409	FF	22			E50A0020 6F007500
57	8000	CTR_IN									0	0		0	0	0	0	0
58	8010	CTR_OUT									0	0		0	0	0	0	0
59	8010	CTR_OUT	Get Manufacturor Str Descr		0	80		6	301	409	FF	22			E50A0020 6F007500
60	8000	CTR_IN									0	0		0	0	0	0	0
61	8010	CTR_OUT									0	0		0	0	0	0	0
62	8010	CTR_OUT	Get Product Str Descriptor		0	80		6	302	409	FF	22			E50A0020 6F007500
63	8000	CTR_IN									0	0		0	0	0	0	0
64	8010	CTR_OUT									0	0		0	0	0	0	0
65	8010	CTR_OUT	Get Product Str Descriptor		0	80		6	302	409	FF	22			E50A0020 6F007500
66	8000	CTR_IN									0	0		0	0	0	0	0
67	8010	CTR_OUT									0	0		0	0	0	0	0
68	8010	CTR_OUT	CDC: Get Line Coding			0	A1		21	0	2	7	7			AC050020
69	8000	CTR_IN									0	0		0	0	0	0	0
70	8010	CTR_OUT									0	0		0	0	0	0	0
71	8010	CTR_OUT	CDC: Set Control Line State		0	21		22	0	2	0	0			21220000 02000000
72	8000	CTR_IN									0	0		0	0	0	0	0
73	8010	CTR_OUT	CDC: Set Line Coding			0	21		20	0	2	7	8			21200000 02000700
74	8010	CTR_OUT									0	0		0	0	0	0	0			000000E4
75	8000	CTR_IN									0	0		0	0	0	0	0
76	8010	CTR_OUT	CDC: Get Line Coding			0	A1		21	0	2	7	7			AC050020 000000E4
77	8000	CTR_IN									0	0		0	0	0	0	0
78	8010	CTR_OUT									0	0		0	0	0	0	0
79	8010	CTR_OUT	Get Lang Str Descriptor			0	80		6	300	0	4	4			820C0020 4F005000
80	8000	CTR_IN									0	0		0	0	0	0	0
81	8010	CTR_OUT									0	0		0	0	0	0	0
82	8010	CTR_OUT	Get Manufacturor Str Descr		0	80		6	301	409	FF	22			E50A0020 6F007500
83	8000	CTR_IN									0	0		0	0	0	0	0
84	8010	CTR_OUT									0	0		0	0	0	0	0
85	8010	CTR_OUT	Get Lang Str Descriptor			0	80		6	300	0	4	4			820C0020 55005600
86	8000	CTR_IN									0	0		0	0	0	0	0
87	8010	CTR_OUT									0	0		0	0	0	0	0
88	8010	CTR_OUT	Get Product Str Descriptor		0	80		6	302	409	FF	22			E50A0020 6F007500
89	8000	CTR_IN									0	0		0	0	0	0	0
90	8010	CTR_OUT									0	0		0	0	0	0	0
91	8010	CTR_OUT	Get Descriptor					0	80		6	600	0	A	8			80060006 00000A00
92	8010	CTR_OUT	Get Descriptor					0	80		6	700	0	9	8			80060007 00000900
93	8010	CTR_OUT	Get Lang Str Descriptor			0	80		6	300	0	1FE	4			820C0020 5D005E00
94	8000	CTR_IN									0	0		0	0	0	0	0
95	8010	CTR_OUT									0	0		0	0	0	0	0
96	8010	CTR_OUT	Get Manufacturor Str Descr		0	80		6	301	409	1FE	22			E50A0020 6F007500
97	8000	CTR_IN									0	0		0	0	0	0	0
98	8010	CTR_OUT									0	0		0	0	0	0	0
99	8010	CTR_OUT	Get Product Str Descriptor		0	80		6	302	409	1FE	22			E50A0020 6F007500
100	8000	CTR_IN									0	0		0	0	0	0	0
101	8010	CTR_OUT									0	0		0	0	0	0	0
102	8010	CTR_OUT									0	80		6	304	409	1FE	8			80060403 0904FE01
103	8010	CTR_OUT									0	80		6	305	409	1FE	8			80060503 0904FE01
104	8010	CTR_OUT	Get CDC Str Descriptor			0	80		6	306	409	1FE	2A			E50A0020 6F007500
105	8000	CTR_IN									0	0		0	0	0	0	0
106	8010	CTR_OUT									0	0		0	0	0	0	0
107	8010	CTR_OUT									0	80		6	307	409	1FE	2C			E50A0020 6F007500
108	8000	CTR_IN									0	0		0	0	0	0	0
109	8010	CTR_OUT									0	0		0	0	0	0	0
110	8010	CTR_OUT									0	80		6	308	409	1FE	8			80060803 0904FE01


*/
#endif

#pragma GCC pop_options
