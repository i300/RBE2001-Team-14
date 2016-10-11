#include <BTComms.h>
#include "Utilities.h"

#ifndef BLUETOOTH_H
#define BLUETOOTH_H

struct Message {
  unsigned int type;
  unsigned int source;
  unsigned int destination;
  unsigned int data[];
  unsigned int checksum;
};

class FieldController {
public:
  typedef enum {
    kHighRadiation = 0xFF,
    kLowRadiation = 0x2C,
    kNoRadiation = 1
  } RadiationStatus;

  typedef enum {
    kReserved,              // 0
  	kStorageAvailability,   // 1
  	kSupplyAvailability,    // 2
  	kRadiationAlert,        // 3
  	kStopMovement,          // 4
  	kResumeMovement,        // 5
  	kRobotStatus,           // 6
  	kHeartbeat              // 7
  } MessageType;

  FieldController(uint8 RobotID);
  void setup();
  void update();

  void sendHeartbeat();

  void printStatus();

  bool8 getStopped();
  bool8 getStorageAvailability(int8 tube);
  bool8 getSupplyAvailability(int8 tube);
  int8 getClosestOpenStorage(int8 currentReactor);
  int8 getClosestFullSupply(int8 currentReactor);

  RadiationStatus status;

  // for testing
  unsigned long lastRadiationAlertTime = 0;

private:
  byte storageAvailability;
  byte supplyAvailability;
  RadiationStatus radiationStatus;
  bool8 stopped;
  uint8 _robotID;

  unsigned long heartbeatTimer;
  unsigned long radiationStatusTimer;

  const int TYPE_INDEX = 0x00;
  const int SOURCE_INDEX = 0x01;
  const int DESTINATION_INDEX = 0x02;
  const int DATA_OR_CHECKSUM_INDEX = 0x03;

  bool8 getAvailability(byte container, int8 index);
  int8 getClosestAvailability(byte container, int8 currentReactor);
};

#endif
