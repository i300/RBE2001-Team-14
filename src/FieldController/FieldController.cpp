#include "FieldController.hpp"

BTComms field;

/**
 * Initializer
 */
FieldController::FieldController() {
  storageAvailability = 0;
  supplyAvailability = 0;
  radiationStatus = kNoRadiation;
  stopped = false;
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
  // Send heartbeat

  // Read incoming messages
  while (field.read()) {
    Message message;
    message.type = field.getMessageByte(TYPE_INDEX);
    message.source = field.getMessageByte(SOURCE_INDEX);
    message.destination = field.getMessageByte(DESTINATION_INDEX);

    // Ignore messages with reserved types
    if (message.type > kReserved && message.type < 0x08) break;

    switch (message.type) {
      case kStorageAvailability:
        storageAvailability = field.getMessageByte(DATA_OR_CHECKSUM_INDEX);
        break;
      case kSupplyAvailability:
        supplyAvailability = field.getMessageByte(DATA_OR_CHECKSUM_INDEX);
        break;
      case kRadiationAlert:
        if (field.getMessageByte(DATA_OR_CHECKSUM_INDEX) == 0x2C) {
          radiationStatus = kLowRadiation;
        } else if (field.getMessageByte(DATA_OR_CHECKSUM_INDEX) == 0xFF) {
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
 * Send heartbeat message to field
 */
void FieldController::sendHeartbeat() {
	field.writeMessage(kHeartbeat, 0x0a, 0x00);
}

/** DEBUG
 * Prints stored information about the field to serial
 */
void FieldController::printStatus() {
  Serial.print("Robot Stopped: "); Serial.println(stopped);
  Serial.print("Radiation Level: "); Serial.println(radiationStatus);

  Serial.print("Storage Availability: ");
  Serial.print(getStorageAvailability(1)); Serial.print(" ");
  Serial.print(getStorageAvailability(2)); Serial.print(" ");
  Serial.print(getStorageAvailability(3)); Serial.print(" ");
  Serial.println(getStorageAvailability(4));

  Serial.print("Supply Availability: ");
  Serial.print(getSupplyAvailability(1)); Serial.print(" ");
  Serial.print(getSupplyAvailability(2)); Serial.print(" ");
  Serial.print(getSupplyAvailability(3)); Serial.print(" ");
  Serial.println(getSupplyAvailability(4));

  Serial.println();Serial.println();
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

/**
 * @returns True if the Robot should be stopped, false otherwise
 */
bool8 FieldController::getStopped() {
  return stopped;
}