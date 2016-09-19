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
    kHighRadiation,
    kLowRadiation
  } RadiationStatus;

  typedef enum {
    kReserved,
  	kStorageAvailability,
  	kSupplyAvailability,
  	kRadiationAlert,
  	kStopMovement,
  	kResumeMovement,
  	kRobotStatus,
  	kHeartbeat
  } MessageType;

  FieldController();
  void setup();
  void update();

  bool8 getStopped();
  bool8 getStorageAvailability(int8 tube);
  bool8 getSupplyAvailability(int8 tube);

private:
  BTComms field;

  byte storageAvailability;
  byte supplyAvailability;
  RadiationStatus radiationStatus;
  bool8 stopped;

  const int TYPE_INDEX = 0x01;
  const int SOURCE_INDEX = 0x02;
  const int DESTINATION_INDEX = 0x03;

  bool8 getAvailability(byte container, int8 index);
};

#endif
