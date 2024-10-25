#pragma once

#include "initialisation.h"
#include "USBHandler.h"

class USB;

class MidiHandler : public USBHandler {
public:
	MidiHandler(USBMain* usb, uint8_t inEP, uint8_t outEP, int8_t interface) : USBHandler(usb, inEP, outEP, interface) {
		outBuff = xfer_buff;
	}

	void DataIn() override;
	void DataOut() override;
	void ActivateEP() override;
	void ClassSetup(usbRequest& req) override;
	void ClassSetupData(usbRequest& req, const uint8_t* data) override;
	uint32_t GetInterfaceDescriptor(const uint8_t** buffer) override;

	void serialHandler(uint32_t data);

	enum MIDIType {Unknown = 0, NoteOn = 0x9, NoteOff = 0x8, PolyPressure = 0xA, ControlChange = 0xB,
		ProgramChange = 0xC, ChannelPressure = 0xD, PitchBend = 0xE, System = 0xF };

	struct MidiNote {
		MidiNote(uint8_t n = 0, uint8_t v = 0) : noteValue(n), velocity(v) {};

		uint8_t noteValue;		// MIDI note value
		uint8_t velocity;
	};

	static const uint8_t Descriptor[];
	static constexpr uint8_t MidiClassDescSize = 50;		// size of just the MIDI class definition (excluding interface descriptors)

	uint16_t pitchBend = 8192;								// Pitchbend amount in raw format (0 - 16384)
	const float pitchBendSemiTones = 12.0f;					// Number of semitones for a full pitchbend

private:

	void midiEvent(const uint32_t data);
	void QueueInc();

	uint32_t xfer_buff[64] __attribute__ ((aligned (4)));	// Receive data buffer - must be aligned to allow copying to other structures

	// Struct for holding incoming USB MIDI data
	union MidiData {
		MidiData(uint32_t d) : data(d) {};
		MidiData()  {};

		uint32_t data;
		struct {
			uint8_t CIN : 4;
			uint8_t cable : 4;
			uint8_t chn : 4;
			uint8_t msg : 4;
			uint8_t db1;
			uint8_t db2;
		};
	};

	static constexpr uint16_t SerialQueueSize = 256;
	uint8_t Queue[SerialQueueSize];			// hold incoming serial MIDI bytes
	uint8_t QueueRead = 0;
	uint8_t QueueWrite = 0;
	uint8_t QueueSize = 0;

};
