#include "USB.h"
#include "CDCHandler.h"
#include "Calib.h"
#include "ledManager.h"
#include <stdio.h>
#include <charconv>

uint32_t flashBuff[8192];

// Check if a command has been received from USB, parse and action as required
void CDCHandler::ProcessCommand()
{
	if (!cmdPending) {
		return;
	}

	std::string_view cmd {comCmd};

	// Provide option to switch to USB DFU mode - this allows the MCU to be programmed with STM32CubeProgrammer in DFU mode
	if (state == serialState::dfuConfirm) {
		if (cmd.compare("y") == 0 || cmd.compare("Y") == 0) {
			usb->SendString("Switching to DFU Mode ...\r\n");
			uint32_t old = SysTickVal;
			while (SysTickVal < old + 100) {};		// Give enough time to send the message
			JumpToBootloader();
		} else {
			state = serialState::pending;
			usb->SendString("Upgrade cancelled\r\n");
		}


	} else if (calib.calibrating) {						// Send command to calibration process
		calib.Calibrate(cmd[0]);

	} else if (cmd.compare("info") == 0) {				// Print diagnostic information
		extern uint32_t underrun;			// Diagnostics from interrupt hander and wavetable output
		printf("\r\nMountjoy Spectralist v1.0\r\n\r\n"
				"Build Date: %s %s\r\n"
				"Calibration base pitch: %.2f\r\n"
				"Calibration pitch divider: %.2f\r\n"
				"Config sector: %lu; address: %p\r\n"
				"Sample buffer underrun: %lu\r\n"
				"\r\n"
				, __DATE__, __TIME__,
				calib.cfg.pitchBase,
				-1.0f / calib.cfg.pitchMult,
				config.currentSector,
				config.flashConfigAddr + config.currentSettingsOffset / 4,
				underrun
				);


	} else if (cmd.compare("help") == 0) {

		usb->SendString("Mountjoy Spectralist\r\n"
				"\r\nSupported commands:\r\n"
				"info        -  Show diagnostic information\r\n"
				"calib       -  Start calibration\r\n"
				"dfu         -  USB firmware upgrade\r\n"
				"clearconfig -  Erase configuration and restart\r\n"
				"saveconfig  -  Immediately save config\r\n"
				"reboot      -  Reboot module\r\n"
				"writei2c:RR,VV - Write value 0xVV to led register 0xRR\r\n"
				"\r\n"

#if (USB_DEBUG)
				"\r\n"
				"usbdebug    -  Start USB debugging\r\n"
				"\r\n"
#endif
		);


#if (USB_DEBUG)
	} else if (cmd.compare("usbdebug") == 0) {				// Configure gate LED
		extern bool USBDebug;
		USBDebug = true;
		usb->SendString("Press link button to dump output\r\n");
#endif

	} else if (cmd.compare("dfu") == 0) {						// USB DFU firmware upgrade
		printf("Start DFU upgrade mode? Press 'y' to confirm.\r\n");
		state = serialState::dfuConfirm;


	} else if (cmd.compare("reboot") == 0) {					// Reboot module
		Reboot();

	} else if (cmd.compare(0, 9, "writei2c:") == 0) {			// write i2c register

		uint8_t regNo, value;
		auto res = std::from_chars(cmd.data() + cmd.find(":") + 1, cmd.data() + cmd.size(), regNo, 16);
		if (res.ec == std::errc()) {			// no error
			auto res = std::from_chars(cmd.data() + cmd.find(",") + 1, cmd.data() + cmd.size(), value, 16);
			if (res.ec == std::errc()) {			// no error

				ledManager.WriteRegister(regNo, value);		// Register is 0xF0, but first bit is autoincrement

				printf("I2C write: Register: %#04x Value: %#04x\r\n", regNo, value);
			} else {
				usb->SendString("Invalid value\r\n");
			}
		} else {
			usb->SendString("Invalid register\r\n");
		}

	} else if (cmd.compare(0, 8, "readi2c:") == 0) {			// read i2c register

		uint8_t regNo;
		auto res = std::from_chars(cmd.data() + cmd.find(":") + 1, cmd.data() + cmd.size(), regNo, 16);
		if (res.ec == std::errc()) {			// no error
			uint8_t value = ledManager.ReadRegister(regNo);		// Register is 0xF0, but first bit is autoincrement
			printf("I2C write: Register: %#04x Value: %#04x\r\n", regNo, value);
		} else {
			usb->SendString("Invalid register\r\n");
		}

	} else if (cmd.compare("dmaled") == 0) {						// Send led dma data
		ledManager.DMASend();

	} else if (cmd.compare("calib") == 0) {						// Start calibration process
		calib.Calibrate('s');


	} else if (cmd.compare("clearconfig") == 0) {				// Erase config from internal flash
		printf("Clearing config and restarting ...\r\n");
		config.EraseConfig();
		DelayMS(10);
		Reboot();


	} else if (cmd.compare("saveconfig") == 0) {				// Immediate config save
		config.SaveConfig(true);


	} else {
		printf("Unrecognised command: %s\r\nType 'help' for supported commands\r\n", cmd.data());
	}

	cmdPending = false;
}


void CDCHandler::DataIn()
{
	if (inBuffSize > 0 && inBuffSize % USB::ep_maxPacket == 0) {
		inBuffSize = 0;
		EndPointTransfer(Direction::in, inEP, 0);				// Fixes issue transmitting an exact multiple of max packet size (n x 64)
	}
}


// As this is called from an interrupt assign the command to a variable so it can be handled in the main loop
void CDCHandler::DataOut()
{
	// Check if sufficient space in command buffer
	const uint32_t newCharCnt = std::min(outBuffCount, maxCmdLen - 1 - buffPos);

	strncpy(&comCmd[buffPos], (char*)outBuff, newCharCnt);
	buffPos += newCharCnt;

	// Check if cr has been sent yet
	if (comCmd[buffPos - 1] == 13 || comCmd[buffPos - 1] == 10 || buffPos == maxCmdLen - 1) {
		comCmd[buffPos - 1] = '\0';
		cmdPending = true;
		buffPos = 0;
	}
}


void CDCHandler::ActivateEP()
{
	EndPointActivate(USB::CDC_In,   Direction::in,  EndPointType::Bulk);			// Activate CDC in endpoint
	EndPointActivate(USB::CDC_Out,  Direction::out, EndPointType::Bulk);			// Activate CDC out endpoint
	EndPointActivate(USB::CDC_Cmd,  Direction::in,  EndPointType::Interrupt);		// Activate Command IN EP

	EndPointTransfer(Direction::out, USB::CDC_Out, USB::ep_maxPacket);
}


void CDCHandler::ClassSetup(usbRequest& req)
{
	if (req.RequestType == DtoH_Class_Interface && req.Request == GetLineCoding) {
		SetupIn(req.Length, (uint8_t*)&lineCoding);
	}

	if (req.RequestType == HtoD_Class_Interface && req.Request == SetLineCoding) {
		// Prepare to receive line coding data in ClassSetupData
		usb->classPendingData = true;
		EndPointTransfer(Direction::out, 0, req.Length);
	}
}


void CDCHandler::ClassSetupData(usbRequest& req, const uint8_t* data)
{
	// ClassSetup passes instruction to set line coding - this is the data portion where the line coding is transferred
	if (req.RequestType == HtoD_Class_Interface && req.Request == SetLineCoding) {
		lineCoding = *(LineCoding*)data;
	}
}

int32_t CDCHandler::ParseInt(const std::string_view cmd, const char precedingChar, const int32_t low, const int32_t high) {
	int32_t val = -1;
	const int8_t pos = cmd.find(precedingChar);		// locate position of character preceding
	if (pos >= 0 && std::strspn(&cmd[pos + 1], "0123456789-") > 0) {
		val = std::stoi(&cmd[pos + 1]);
	}
	if (high > low && (val > high || val < low)) {
		printf("Must be a value between %ld and %ld\r\n", low, high);
		return low - 1;
	}
	return val;
}


float CDCHandler::ParseFloat(const std::string_view cmd, const char precedingChar, const float low = 0.0f, const float high = 0.0f) {
	float val = -1.0f;
	const int8_t pos = cmd.find(precedingChar);		// locate position of character preceding
	if (pos >= 0 && std::strspn(&cmd[pos + 1], "0123456789.") > 0) {
		val = std::stof(&cmd[pos + 1]);
	}
	if (high > low && (val > high || val < low)) {
		printf("Must be a value between %f and %f\r\n", low, high);
		return low - 1.0f;
	}
	return val;
}


// Descriptor definition here as requires constants from USB class
const uint8_t CDCHandler::Descriptor[] = {
	// IAD Descriptor - Interface association descriptor for CDC class
	0x08,									// bLength (8 bytes)
	USB::IadDescriptor,						// bDescriptorType
	USB::CDCCmdInterface,					// bFirstInterface
	0x02,									// bInterfaceCount
	0x02,									// bFunctionClass (Communications and CDC Control)
	0x02,									// bFunctionSubClass
	0x01,									// bFunctionProtocol
	USB::CommunicationClass,				// String Descriptor

	// Interface Descriptor
	0x09,									// bLength: Interface Descriptor size
	USB::InterfaceDescriptor,				// bDescriptorType: Interface
	USB::CDCCmdInterface,					// bInterfaceNumber: Number of Interface
	0x00,									// bAlternateSetting: Alternate setting
	0x01,									// bNumEndpoints: 1 endpoint used
	0x02,									// bInterfaceClass: Communication Interface Class
	0x02,									// bInterfaceSubClass: Abstract Control Model
	0x01,									// bInterfaceProtocol: Common AT commands
	USB::CommunicationClass,				// iInterface

	// Header Functional Descriptor
	0x05,									// bLength: Endpoint Descriptor size
	USB::ClassSpecificInterfaceDescriptor,	// bDescriptorType: CS_INTERFACE
	0x00,									// bDescriptorSubtype: Header Func Desc
	0x10,									// bcdCDC: spec release number
	0x01,

	// Call Management Functional Descriptor
	0x05,									// bFunctionLength
	USB::ClassSpecificInterfaceDescriptor,	// bDescriptorType: CS_INTERFACE
	0x01,									// bDescriptorSubtype: Call Management Func Desc
	0x00,									// bmCapabilities: D0+D1
	0x01,									// bDataInterface: 1

	// ACM Functional Descriptor
	0x04,									// bFunctionLength
	USB::ClassSpecificInterfaceDescriptor,	// bDescriptorType: CS_INTERFACE
	0x02,									// bDescriptorSubtype: Abstract Control Management desc
	0x02,									// bmCapabilities

	// Union Functional Descriptor
	0x05,									// bFunctionLength
	USB::ClassSpecificInterfaceDescriptor,	// bDescriptorType: CS_INTERFACE
	0x06,									// bDescriptorSubtype: Union func desc
	0x00,									// bMasterInterface: Communication class interface
	0x01,									// bSlaveInterface0: Data Class Interface

	// Endpoint 2 Descriptor
	0x07,									// bLength: Endpoint Descriptor size
	USB::EndpointDescriptor,				// bDescriptorType: Endpoint
	USB::CDC_Cmd,							// bEndpointAddress
	USB::Interrupt,							// bmAttributes: Interrupt
	0x08,									// wMaxPacketSize
	0x00,
	0x10,									// bInterval

	//---------------------------------------------------------------------------

	// Data class interface descriptor
	0x09,									// bLength: Endpoint Descriptor size
	USB::InterfaceDescriptor,				// bDescriptorType:
	USB::CDCDataInterface,					// bInterfaceNumber: Number of Interface
	0x00,									// bAlternateSetting: Alternate setting
	0x02,									// bNumEndpoints: Two endpoints used
	0x0A,									// bInterfaceClass: CDC
	0x00,									// bInterfaceSubClass:
	0x00,									// bInterfaceProtocol:
	0x00,									// iInterface:

	// Endpoint OUT Descriptor
	0x07,									// bLength: Endpoint Descriptor size
	USB::EndpointDescriptor,				// bDescriptorType: Endpoint
	USB::CDC_Out,							// bEndpointAddress
	USB::Bulk,								// bmAttributes: Bulk
	LOBYTE(USB::ep_maxPacket),				// wMaxPacketSize:
	HIBYTE(USB::ep_maxPacket),
	0x00,									// bInterval: ignore for Bulk transfer

	// Endpoint IN Descriptor
	0x07,									// bLength: Endpoint Descriptor size
	USB::EndpointDescriptor,				// bDescriptorType: Endpoint
	USB::CDC_In,							// bEndpointAddress
	USB::Bulk,								// bmAttributes: Bulk
	LOBYTE(USB::ep_maxPacket),				// wMaxPacketSize:
	HIBYTE(USB::ep_maxPacket),
	0x00,									// bInterval: ignore for Bulk transfer
};


uint32_t CDCHandler::GetInterfaceDescriptor(const uint8_t** buffer) {
	*buffer = Descriptor;
	return sizeof(Descriptor);
}
