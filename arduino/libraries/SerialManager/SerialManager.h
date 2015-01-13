/*
  SerialManager.h - Library for doing packetized serial comm with Arduinos.
  Created by Sigurdur Orn, May 23, 2010.
  siggi@mit.edu
 */

#ifndef SerialManager_h
#define SerialManager_h

#include "ByteBuffer.h"
#include "WProgram.h"

typedef void (*voidFuncPtr)(ByteBuffer*);

#if defined(__AVR_ATmega8__)
	#define UCSRA	UCSRA
	#define UDRE	UDRE
#else
	#define UCSRA	UCSR0A
	#define UDRE	UDRE0
#endif

#if defined(__AVR_ATmega1280__)
	#define UCSRA1	UCSR1A
	#define UCSRA2	UCSR2A
	#define UCSRA3	UCSR3A
#endif


class SerialManager
{
public:
	SerialManager(unsigned int in_buf_size, unsigned int out_buf_size);
	void init(int serial_port, int baud_rate);
	void setPacketHandler(void (*rx_func)(ByteBuffer*));

	void update();
	bool isBusySending();

	int sendSerialByte(byte b);
	int sendSerialPacket(ByteBuffer* packet);
	int sendRawSerial(ByteBuffer* packet);

private:
	void handleIncomingByte(byte incoming);
	void handlePacketDefault(ByteBuffer* packet);

	voidFuncPtr handlePacketFunction;

	byte _serial_port;
	ByteBuffer* incoming_buffer;
	ByteBuffer* outgoing_buffer;
	ByteBuffer* temp_buffer;
	byte serial_in_checksum;
	byte byte1;
	byte byte2;
	byte byte3;
	byte byte4;
};

#endif

