#include "Arduino.h"
#include "Utilities.h"
#include "RobotMap.h"

#include "FieldController/FieldController.hpp"

#include "Subsystems/DriveTrain/DriveTrain.hpp"
#include "Subsystems/RodGrabber/RodGrabber.hpp"

#include "RobotTask/RobotTask.hpp"

// Sensors and controllers
FieldController *fieldController;

// Subsystems
DriveTrain *driveTrain;
RodGrabber *rodGrabber;

// Task stuff
RobotTask currentTask;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");

  currentTask = RobotTask();

  fieldController = new FieldController();
  fieldController->setup();

  driveTrain = new DriveTrain(PIN_MOTOR_LEFT, PIN_MOTOR_RIGHT, DriveTrainInvertedSide::INVERTED_LEFT);
  rodGrabber = new RodGrabber(PIN_MOTOR_GRABBER, PIN_SERVO_GRABBER, PIN_SENSOR_POT);
}

long nextTime = millis() + 2500;
bool8 grabbed = false;

void loop() {
  // Update bluetooth controller
  fieldController->update();

  // Stop all actions if robot is stopped
  if (fieldController->getStopped()) {
    return;
  }

  // Update Subsystems
  rodGrabber->update();

  // TEST
  if (millis() >= nextTime) {
    //Serial.println(nextTime);

    if (grabbed) {
      rodGrabber->grab();
      delay(500);
      rodGrabber->moveUp();

      grabbed = false;
      nextTime = millis() + 5000;
    } else {
      rodGrabber->release();
      delay(2000);
      rodGrabber->moveDown();

      grabbed = true;
      nextTime = millis() + 5000;
    }
  }

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
