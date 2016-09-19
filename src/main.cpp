#include "Arduino.h"
#include "RobotMap.h"
#include "FieldController/FieldController.hpp"
#include "Subsystems/DriveTrain/DriveTrain.hpp"

DriveTrain *driveTrain;
FieldController *fieldController;

typedef enum {

} ROBOT_STATE;

void setup() {
  driveTrain = new DriveTrain(PIN_MOTOR_LEFT, PIN_MOTOR_RIGHT, DriveTrainInvertedSide::INVERTED_LEFT);
}

void loop() {
  // Update bluetooth controller
  fieldController->update();

  if (fieldController->getStopped()) {

  }
}
