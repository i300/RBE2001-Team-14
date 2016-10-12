#include "BTComms.h"

#include "Arduino.h"

/**
 * Bluetooth communications constructor
 */
BTComms::BTComms() {
	messageIndex = 0;
	messageLength = 0;
	BTstate = kLookingForStart;
}

/**
 * Code that is called from the arduino setup() function
 * This initializes things that cannot be set up from the constructor.
 */
void BTComms::setup() {
  Serial3.begin(115200);
}

/**
 * Send a message to the RCS that has 3 values
 */
void BTComms::writeMessage(unsigned char type, unsigned char source, unsigned char dest) {
  Serial3.write(kMessageStart);
  Serial3.write(5);
  Serial3.write(type);
  Serial3.write(source);
  Serial3.write(dest);
  Serial3.write(0xff - (type + source + dest + 5));
}

/**
 * Send a message to the RCS that has 4 values
 */
void BTComms::writeMessage(unsigned char type, unsigned char source, unsigned char dest, unsigned char data) {
  Serial3.write(kMessageStart);
  Serial3.write(6);
  Serial3.write(type);
  Serial3.write(source);
  Serial3.write(dest);
	Serial3.write(data);
  Serial3.write(0xff - (type + source + dest + data + 6));
}

/**
 * Get the length of the currently received message
 * @returns int The number of bytes in the received message
 */
int BTComms::getMessageLength() {
  return messageLength;
}

/**
 * Get a byte from the current message
 * Retrieve a byte from the currently received message. Only a single message is
 * remembered at any time, so you have to call read(), notice that there is a message,
 * and then do something with the message bytes.
 * @param index The offset (zero-based) to the byte to be returned
 * @returns unsigned char The byte that is at the specified index
 */
unsigned char BTComms::getMessageByte(unsigned index) {
  return message[index];
}

/**
 * Read a message from Bluetooth
 * This method reads messages from Bluetooth by looking for the message start byte, then
 * reading the message length and data.
 *
 * You should probably modify this to ignore messages with invalid checksums!
 */
bool BTComms::read() {
	uint8_t calculatedChecksum = 255;
  while (Serial3.available()) {
    unsigned inByte = Serial3.read();
    switch (BTstate) {
      case kLookingForStart:
        if (inByte != kMessageStart)
        	break;
        BTstate = kReadingMessageLength;
				Serial.println("Message Start");
        break;
      case kReadingMessageLength:
        messageLength = inByte - 1;
        messageIndex = 0;
        BTstate = kReadMessage;
				Serial.println(messageLength, HEX);
        break;
      case kReadMessage:
				Serial.print(String(messageIndex) + ": ");
				if (messageIndex == 3) Serial.println(inByte, BIN);
				else Serial.println(inByte, HEX);
        message[messageIndex++] = inByte;
        if (messageIndex >= messageLength) {
					calculatedChecksum -= messageLength + 1;
					for (int i=0; i < messageLength-1; i++) {
						calculatedChecksum -= message[i];
					}
          BTstate = kLookingForStart;
					Serial.print("C: "); Serial.println(calculatedChecksum, HEX);
					Serial.println("Message End");
					if (calculatedChecksum == inByte) {
	          return true;
					} else {
						return false;
					}
        }
        break;
       default:
        Serial.println("Invalid state");
    }
  }
  return false;
}
