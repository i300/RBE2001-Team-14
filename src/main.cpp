#include "Arduino.h"
#include "RobotMap.h"
#include "FieldController/FieldController.hpp"
#include "Subsystems/DriveTrain/DriveTrain.hpp"
#include "RobotTask/RobotTask.hpp"

DriveTrain *driveTrain;
FieldController *fieldController;

RobotTask currentTask;

long heartbeatTimer = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");

  currentTask = RobotTask();

  fieldController = new FieldController();
  fieldController->setup();

  //driveTrain = new DriveTrain(PIN_MOTOR_LEFT, PIN_MOTOR_RIGHT, DriveTrainInvertedSide::INVERTED_LEFT);

  heartbeatTimer = millis() + 1000;
}

void loop() {
  // Update bluetooth controller
  fieldController->update();

  // Stop all actions if robot is stopped
  if (fieldController->getStopped()) {
    return;
  }

  /*if (millis() > heartbeatTimer) {
    heartbeatTimer = millis() + 1000;
    fieldController->sendHeartbeat();
  }*/

  // Update current task
  currentTask.update();

  // Update state machine if task is finished
  RobotTaskType taskType = currentTask.getType();
  if (currentTask.isFinished()) {
    switch (taskType) {

      case NO_TASK:
        //currentTask = new PickupFromReactorTask();
        break;
      case AQUIRE_NEW_ROD:
        //currentTask = new DropOffAtReactorTask();
        break;
      case STORE_USED_ROD:
        //currentTask = new AquireRodTask();
        break;
      case PICKUP_FROM_REACTOR:
        //currentTask = new StoreUsedRodTask();
        break;
      case DROP_OFF_AT_REACTOR:
        //currentTask = new PickupFromReactorTask();
        break;

    }
  }

}
