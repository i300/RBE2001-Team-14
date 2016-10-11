#include "FieldController.hpp"

BTComms field;

/**
 * Initializer
 */
FieldController::FieldController(uint8 robotID) {
  storageAvailability = 0;
  supplyAvailability = 0;
  radiationStatus = kNoRadiation;
  heartbeatTimer = millis() + 1000;
  radiationStatusTimer = millis() + 1500;
  _robotID = robotID;
  status = kNoRadiation;
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
  unsigned long currentTime = millis();

  // Send a heartbeat every second.
  // Send a radiation alert 500 ms after every heartbeat

  // Send heartbeat
  if (currentTime > heartbeatTimer) {
    heartbeatTimer = currentTime + 1000;
    radiationStatusTimer = currentTime + 500;
    sendHeartbeat();
  }

  // Send radiation status
  if (currentTime > radiationStatusTimer) {
    // Reset radiationStatusTimer to 10 seconds in the future. This is so
    // the robot doesnt spam the field with messages. The actual
    // radiationStatusTimer will be after sending a heartbeat
    radiationStatusTimer = currentTime + 10000;

    lastRadiationAlertTime = currentTime;

    // Send status
    if (status == kHighRadiation) {
      field.writeMessage(kRadiationAlert, _robotID, kHighRadiation);
    } else if (status == kLowRadiation) {
      field.writeMessage(kRadiationAlert, _robotID, kLowRadiation);
    }
  }

  // Read incoming messages
  while (field.read()) {
    Message message;
    message.type = field.getMessageByte(TYPE_INDEX);
    message.source = field.getMessageByte(SOURCE_INDEX);
    message.destination = field.getMessageByte(DESTINATION_INDEX);

    // Ignore messages with reserved types
    if (message.type <= kReserved && message.type >= 0x08) break;

    // Ignore messages for other robots
    if (message.destination != _robotID && message.destination != 0) break;

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
	field.writeMessage(kHeartbeat, _robotID, 0x00);
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
  return getAvailability(storageAvailability, 4 - tube);
}

/**
 * Get the availability of a tube in the supply container
 * @returns 0 = No Rod Present, 1 = Rod Present
 */
bool8 FieldController::getSupplyAvailability(int8 tube) {
  return getAvailability(supplyAvailability, 4 - tube);
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
  return (container & (1 << (tubeIndex))) >> (tubeIndex);
}

/**
 * @returns True if the Robot should be stopped, false otherwise
 */
bool8 FieldController::getStopped() {
  return stopped;
}

int8 FieldController::getClosestOpenStorage(int8 currentReactor) {
  return getClosestAvailability(storageAvailability, currentReactor);
}

int8 FieldController::getClosestFullSupply(int8 currentReactor) {
  return getClosestAvailability(supplyAvailability, currentReactor);
}

int8 FieldController::getClosestAvailability(byte container, int8 currentReactor) {
  int8 desiredValue = 0;
  if (container == storageAvailability) {
    desiredValue = 0;
  } else if (container == supplyAvailability) {
    desiredValue = 1;
  }
  if (currentReactor == 0) {
    for (int8 i=1; i<=4; i++) {
      if (getAvailability(container, 4 - i) == desiredValue) {
        return i;
      }
    }
  } else if (currentReactor == 1) {
    for (int8 i=4; i>=1; i--) {
      if (getAvailability(container, 4 - i) == desiredValue) {
        return i;
      }
    }
  }

  return 0;
}
