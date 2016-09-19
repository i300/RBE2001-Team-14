#include "FieldController.hpp"

/**
 * Initializer
 */
FieldController::FieldController() {
}

/**
 * Sets up the BTComms communicator
 */
void FieldController::setup() {
  field.setup();
}

/**
 * Read all messages recieved and update the state of the field
 */
void FieldController::update() {
  while (field.read()) {
    Message message;
    message.type = field.getMessageByte(TYPE_INDEX);
    message.source = field.getMessageByte(SOURCE_INDEX);
    message.destination = field.getMessageByte(DESTINATION_INDEX);

    // Ignore messages with reserved types
    if (message.type > kReserved && message.type < 0x08) break;

    unsigned int dataLength = field.getMessageLength() - 4;
    message.checksum = field.getMessageByte(0x05 + dataLength);

    // Checksum is all the bytes from the length to (but not including) the checksum
    unsigned int calculatedChecksum = field.getMessageLength() + message.type + message.source + message.destination;

    // Add data bytes to checksum
    for (unsigned int i=0; i < dataLength; i++) {
      calculatedChecksum += field.getMessageByte(0x05 + i);
    }

    // Break if checksum is invalid
    if (message.checksum == calculatedChecksum) break;

    switch (message.type) {
      case kStorageAvailability:
        storageAvailability = field.getMessageByte(0x05);
        break;
      case kSupplyAvailability:
        supplyAvailability = field.getMessageByte(0x05);
        break;
      case kRadiationAlert:
        if (field.getMessageByte(0x05) == 0x2C) {
          radiationStatus = kLowRadiation;
        } else if (field.getMessageByte(0x05) == 0xFF) {
          radiationStatus = kHighRadiation;
        }
        break;
      case kStopMovement:
        stopped = true;
        break;
      case kResumeMovement:
        stopped = false;
        break;
      case kRobotStatus:
        break;
      case kHeartbeat:
        break;
      default:
        break;
    }
  }
}

/**
 * Get the availability of a tube in the storage container
 * @returns 0 = No Rod Present, 1 = Rod Present
 */
bool8 FieldController::getStorageAvailability(int8 tube) {
  return getAvailability(storageAvailability, tube - 1);
}

/**
 * Get the availability of a tube in the supply container
 * @returns 0 = No Rod Present, 1 = Rod Present
 */
bool8 FieldController::getSupplyAvailability(int8 tube) {
  return getAvailability(supplyAvailability, tube - 1);
}

/**
 * Get the availability of a tube in the specified container
 * @returns 0 = No Rod Present, 1 = Rod Present
 */
bool8 FieldController::getAvailability(byte container, int8 tubeIndex) {
  /*
   * Take the container byte and AND the bitmask based on the tubeIndex, then
   * shift over the resultant to be a single bit.
   *
   * EXAMPLE: Get status of tube 4
   * 1101000 & (00010000) = 0001000
   * 0001000 >> 3 = 1
   * Therefore a rod is present in the tube
   */
  return (container & (1 << (7 - tubeIndex)) >> (7 - tubeIndex));
}

bool8 FieldController::getStopped() {
  return stopped;
}
