#include <Arduino.h>

#ifndef _BTReader
#define _BTReader

/**
 * Low level class to both receive and send message to the field.
 * The writeMessage() method sends messages to the field and the other methods
 * receive message from the field.
 *
 * This class is used by the higher level Messages class to separate the actual
 * byte reading and dealing with checksums from the messages class to make it more
 * understandable.
 */
class BTComms {
  public:
    BTComms();
    void setup();
    int getMessageLength();
    uint8_t getMessageByte(unsigned index);
    bool read();
    void writeMessage(unsigned char type, unsigned char source, unsigned char dest);
    void writeMessage(unsigned char type, unsigned char source, unsigned char dest, unsigned char data);
   private:
    enum BTstate {kLookingForStart, kReadingMessageLength, kReadMessage} BTstate;
    unsigned messageLength;
    uint8_t message[20];
    unsigned messageIndex;
    unsigned char kMessageStart = 0x5f;
};

#endif
