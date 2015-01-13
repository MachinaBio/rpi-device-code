#include <SerialManager.h>
#include <ByteBuffer.h>

SerialManager serial = SerialManager(256, 256);
ByteBuffer send_buffer;

int SEND_MODE = 0;

void setup()
{
  // initialize the serial communication:
  serial.init(0, 9600);
  serial.setPacketHandler(handlePacket);
 
  // Initialize the send buffer that we will use to send data
  send_buffer.init(30);
}

void loop() {
  serial.update();
 
  // If we are not busy sending then lets send something
  if( !serial.isBusySending() ){
    // Send some dummy data  
    send_buffer.clear();
    send_buffer.put(17);
    send_buffer.putInt(300);    
    send_buffer.putLong(-100000);    
    send_buffer.putFloat(3.14);    

    // We can either send a packet with a header and checksum (niiice)
    if(SEND_MODE == 0)
      serial.sendSerialPacket( &send_buffer );
    
    // Or if we are sending to say a device that uses a custom protocol,
    // we can send a raw byte buffer to it
    if(SEND_MODE == 1)
      serial.sendRawSerial( &send_buffer );
    
    // Or if we just want to simply send one byte we can do that,
    if(SEND_MODE == 2)
      serial.sendSerialByte(16);

  }
}

void handlePacket(ByteBuffer* packet){
  // Here we could do anything we want to the data but for now we will just send it back
  send_buffer.clear();
  while( packet->getSize() > 0 )
    send_buffer.put( packet->get() );
  serial.sendSerialPacket( &send_buffer );
}
