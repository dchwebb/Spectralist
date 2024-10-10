#include "MidiHandler.h"
#include "USB.h"

void MidiHandler::DataIn()
{

}

void MidiHandler::DataOut()
{
	// Handle incoming midi command here
	//const uint8_t* outBuffBytes = reinterpret_cast<const uint8_t*>(outBuff);
	//midiEvent(*outBuff);
	/*
	if (!partialSysEx && outBuffCount == 4) {
		midiEvent(*outBuff);

	} else if (partialSysEx || (outBuffBytes[1] == 0xF0 && outBuffCount > 3)) {		// Sysex
		// sysEx will be padded when supplied by usb - add only actual sysEx message bytes to array
		uint16_t sysExCnt = partialSysEx ? 1 : 2;		// If continuing a long sysex command only ignore first (size) byte

		uint16_t i;
		for (i = partialSysEx ? sysExCount : 0; i < sysexMaxSize; ++i) {
			if (outBuffBytes[sysExCnt] == 0xF7) {
				partialSysEx = false;
				break;
			}
			if (sysExCnt >= outBuffCount) {				// Long SysEx command will be received in multiple packets
				partialSysEx = true;
				break;
			}

			sysEx[i] = outBuffBytes[sysExCnt++];

			// remove 1 byte padding at the beginning of each 32 bit word
			if (sysExCnt % 4 == 0) {
				++sysExCnt;
			}
		}
		sysExCount = i;
		if (!partialSysEx) {
			ProcessSysex();
		}
	}
	*/
}


void MidiHandler::ActivateEP()
{
	EndPointActivate(USBMain::Midi_In,   Direction::in,  EndPointType::Bulk);
	EndPointActivate(USBMain::Midi_Out,  Direction::out, EndPointType::Bulk);

	EndPointTransfer(Direction::out, USBMain::Midi_Out, USBMain::ep_maxPacket);
}





void MidiHandler::midiEvent(const uint32_t data)
{
	auto midiData = MidiData(data);
	MidiNote midiNote(midiData.db1, midiData.db2);

	if (midiData.msg == NoteOn || midiData.msg == NoteOff) {
		//voiceManager.NoteOnOff(midiNote.noteValue, midiData.msg == NoteOn);
	} else if (midiData.msg == PitchBend) {
		pitchBend = static_cast<uint16_t>(midiData.db1) + (midiData.db2 << 7);
		//voiceManager.Pitchbend(pitchBend);
	}
}


void MidiHandler::serialHandler(uint32_t data)
{
	Queue[QueueWrite] = data;			// Queue handles MIDI data divided into individual bytes
	++QueueSize;
	++QueueWrite;			// counter will wrap at 256

	MIDIType type = static_cast<MIDIType>(Queue[QueueRead] >> 4);
	uint8_t channel = Queue[QueueRead] & 0x0F;

	//NoteOn = 0x9, NoteOff = 0x8, PolyPressure = 0xA, ControlChange = 0xB, ProgramChange = 0xC, ChannelPressure = 0xD, PitchBend = 0xE, System = 0xF
	while ((QueueSize > 2 && (type == NoteOn || type == NoteOff || type == PolyPressure ||  type == ControlChange ||  type == PitchBend)) ||
		   (QueueSize > 1 && (type == ProgramChange || type == ChannelPressure))) {

		MidiData event;
		event.chn = channel;
		event.msg = (uint8_t)type;

		QueueInc();
		event.db1 = Queue[QueueRead];
		QueueInc();
		if (type == ProgramChange || type == ChannelPressure) {
			event.db2 = 0;
		} else {
			event.db2 = Queue[QueueRead];
			QueueInc();
		}

		midiEvent(event.data);

		type = static_cast<MIDIType>(Queue[QueueRead] >> 4);
		channel = Queue[QueueRead] & 0x0F;
	}

	// Ignore Clock
	if (QueueSize > 0 && Queue[QueueRead] == 0xF8) {
		QueueInc();
	}

	//	handle unknown data in queue
	if (QueueSize > 2 && type != 0x9 && type != 0x8 && type != 0xD && type != 0xE) {
		QueueInc();
	}
}


inline void MidiHandler::QueueInc() {
	--QueueSize;
	++QueueRead;			// counter will wrap at 256
}

/*
Byte 1									|	Byte2		|	Byte 3		|	Byte 4
Cable Number | Code Index Number (CIN)	|	MIDI_0		|	MIDI_1		|	MIDI_2

CIN		MIDI_x Size Description
0x0		1, 2 or 3	Miscellaneous function codes. Reserved for future extensions.
0x1		1, 2 or 3	Cable events. Reserved for future expansion.
0x2		2			Two-byte System Common messages like MTC, SongSelect, etc.
0x3		3			Three-byte System Common messages like SPP, etc.
0x4		3			SysEx starts or continues
0x5		1			Single-byte System Common Message or SysEx ends with following single byte.
0x6		2			SysEx ends with following two bytes.
0x7		3			SysEx ends with following three bytes.
0x8		3			Note-off
0x9		3			Note-on
0xA		3			Poly-KeyPress
0xB		3			Control Change
0xC		2			Program Change
0xD		2			Channel Pressure
0xE		3			PitchBend Change
0xF		1			Single Byte
*/


void MidiHandler::ClassSetup(usbRequest& req)
{
}


void MidiHandler::ClassSetupData(usbRequest& req, const uint8_t* data)
{
}




// Descriptor definition here as requires constants from USB class
const uint8_t MidiHandler::Descriptor[] = {
	// B.3.1 Standard Audio Control standard Interface Descriptor
	0x09,									// length of descriptor in bytes
	USBMain::InterfaceDescriptor,			// interface descriptor type
	USBMain::AudioInterface,				// index of this interface
	0x00,									// alternate setting for this interface
	0x00,									// endpoints excl 0: number of endpoint descriptors to follow
	0x01,									// AUDIO
	0x01,									// AUDIO_Control
	0x00,									// bInterfaceProtocol
	USBMain::AudioClass,					// string index for interface

	// B.3.2 Class-specific AC Interface Descriptor
	0x09,									// length of descriptor in bytes
	USBMain::ClassSpecificInterfaceDescriptor,	// descriptor type
	0x01,									// header functional descriptor
	0x00, 0x01,								// bcdADC
	0x09, 0x00,								// wTotalLength
	0x01,									// bInCollection
	0x01,									// baInterfaceNr[1]

	// B.4 MIDIStreaming Interface Descriptors
	// B.4.1 Standard MS Interface Descriptor
	0x09,									// bLength
	USBMain::InterfaceDescriptor,			// bDescriptorType: interface descriptor
	USBMain::MidiInterface,					// bInterfaceNumber
	0x00,									// bAlternateSetting
	0x02,									// bNumEndpoints
	0x01,									// bInterfaceClass: Audio
	0x03,									// bInterfaceSubClass: MIDIStreaming
	0x00,									// InterfaceProtocol
	USBMain::AudioClass,					// iInterface: String Descriptor

	// B.4.2 Class-specific MS Interface Descriptor
	0x07,									// length of descriptor in bytes
	USBMain::ClassSpecificInterfaceDescriptor,	// bDescriptorType: Class Specific Interface Descriptor
	0x01,									// header functional descriptor
	0x0, 0x01,								// bcdADC
	MidiHandler::MidiClassDescSize, 0,		// wTotalLength

	// B.4.3 MIDI IN Jack Descriptor (Embedded)
	0x06,									// bLength
	USBMain::ClassSpecificInterfaceDescriptor,	// descriptor type
	0x02,									// bDescriptorSubtype: MIDI_IN_JACK
	0x01,									// bJackType: Embedded
	0x01,									// bJackID
	0x00,									// iJack: No String Descriptor

	// Table B4.4 Midi Out Jack Descriptor (Embedded)
	0x09,									// length of descriptor in bytes
	USBMain::ClassSpecificInterfaceDescriptor,	// descriptor type
	0x03,									// MIDI_OUT_JACK descriptor
	0x01,									// bJackType: Embedded
	0x02,									// bJackID
	0x01,									// No of input pins
	0x01,									// ID of the Entity to which this Pin is connected.
	0x01,									// Output Pin number of the Entity to which this Input Pin is connected.
	0x00,									// iJack

	//B.5.1 Standard Bulk OUT Endpoint Descriptor
	0x09,									// bLength
	USBMain::EndpointDescriptor,			// bDescriptorType = endpoint
	USBMain::Midi_Out,						// bEndpointAddress
	USBMain::Bulk,							// bmAttributes: 2:Bulk
	LOBYTE(USBMain::ep_maxPacket),			// wMaxPacketSize
	HIBYTE(USBMain::ep_maxPacket),
	0x00,									// bInterval in ms : ignored for bulk
	0x00,									// bRefresh Unused
	0x00,									// bSyncAddress Unused

	// B.5.2 Class-specific MS Bulk OUT Endpoint Descriptor
	0x05,									// bLength of descriptor in bytes
	0x25,									// bDescriptorType (Audio Endpoint Descriptor)
	0x01,									// bDescriptorSubtype: MS General
	0x01,									// bNumEmbMIDIJack
	0x01,									// baAssocJackID ID of the Embedded MIDI IN Jack.

	//B.6.1 Standard Bulk IN Endpoint Descriptor
	0x09,									// bLength
	USBMain::EndpointDescriptor,			// bDescriptorType = endpoint
	USBMain::Midi_In,						// bEndpointAddress IN endpoint number 3
	USBMain::Bulk,							// bmAttributes: 2: Bulk, 3: Interrupt endpoint
	LOBYTE(USBMain::ep_maxPacket),			// wMaxPacketSize
	HIBYTE(USBMain::ep_maxPacket),
	0x00,									// bInterval in ms
	0x00,									// bRefresh
	0x00,									// bSyncAddress

	// B.6.2 Class-specific MS Bulk IN Endpoint Descriptor
	0x05,									// bLength of descriptor in bytes
	0x25,									// bDescriptorType
	0x01,									// bDescriptorSubtype
	0x01,									// bNumEmbMIDIJack
	0x02,									// baAssocJackID ID of the Embedded MIDI OUT Jack

};


uint32_t MidiHandler::GetInterfaceDescriptor(const uint8_t** buffer) {
	*buffer = Descriptor;
	return sizeof(Descriptor);
}

