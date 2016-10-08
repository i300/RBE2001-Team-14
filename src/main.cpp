#include "Arduino.h"
#include "Utilities.h"
#include "RobotMap.h"

#include "FieldController/FieldController.hpp"
#include <QTRSensors.h>
#include <LiquidCrystal.h>

#include "Subsystems/DriveTrain/DriveTrain.hpp"
#include "Subsystems/RodGrabber/RodGrabber.hpp"

#include "RobotTask/RobotTask.hpp"
#include "RobotTask/Tasks/CalibrationTask.hpp"
#include "RobotTask/Tasks/PickUpFromReactorTask.hpp"
#include "RobotTask/Tasks/AquireRodTask.hpp"
#include "RobotTask/Tasks/StoreRodTask.hpp"

// Sensors and controllers
FieldController *fieldController;
QTRSensorsAnalog *lineSensor;

// Subsystems
DriveTrain *driveTrain;
RodGrabber *rodGrabber;

// Task stuff
RobotTask *currentTask;
int8 currentReactor = 0; // 0 = Reactor A, 1 = Reactor B

// LCD
LiquidCrystal lcd(40, 41, 42, 43, 44, 45);
unsigned long lastWriteTime = 0;
const float lcdFramesPerSecond = 30;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");

  fieldController = new FieldController();
  fieldController->setup();

  lcd.begin(16, 2); // Begin LCD
  delay(50); // Hold up, wait a minute
  lcd.clear(); // Clear LCD

  delay(500);

  lcd.print("Waiting to start...");

  pinMode(PIN_SENSOR_ALIGNMENT, INPUT);
  while (digitalRead(PIN_SENSOR_ALIGNMENT) == 1) {
    delay(10); // wait to start
  }

  lcd.clear();

  uint8 sensorPins[] = { PIN_SENSOR_LINE1, PIN_SENSOR_LINE2, PIN_SENSOR_LINE3,
                         PIN_SENSOR_LINE4, PIN_SENSOR_LINE5, PIN_SENSOR_LINE6,
                         PIN_SENSOR_LINE7, PIN_SENSOR_LINE8 };
  lineSensor = new QTRSensorsAnalog(sensorPins, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, PIN_LINESENSOR_EMITTER);

  driveTrain = new DriveTrain(PIN_MOTOR_LEFT, PIN_MOTOR_RIGHT, PIN_SENSOR_ALIGNMENT,
                              &lcd, lineSensor, DriveTrainInvertedSide::INVERTED_LEFT);
  rodGrabber = new RodGrabber(PIN_MOTOR_GRABBER, PIN_SERVO_GRABBER, PIN_SENSOR_POT);

  rodGrabber->moveUp();
  rodGrabber->grab();

  currentTask = new CalibrationTask(driveTrain, rodGrabber, fieldController);
}

long nextTime = millis() + 2500;
bool8 grabbed = false;

void loop() {
  unsigned long currentTime = millis();

  // Update bluetooth controller
  fieldController->update();

  // Stop all actions if robot is stopped
  if (fieldController->getStopped()) {
    return;
  }

  // Update Subsystems
  rodGrabber->update();

  // Update current task
  currentTask->update();

  // Update state machine if task is finished
  RobotTaskType taskType = currentTask->getType();
  if (currentTask->isFinished()) {
    delete currentTask;

    switch (taskType) {

      case NO_TASK:
        // Nothing to see here...
        break;

      case CALIBRATION:
        currentTask = new PickUpFromReactorTask(driveTrain, rodGrabber, fieldController);
        break;

      case PICKUP_FROM_REACTOR:
        // TODO: Use bluetooth to figure out closest storage that is open
        currentTask = new StoreRodTask(3, driveTrain, rodGrabber, fieldController);
        break;

      case STORE_USED_ROD:
        // TODO: Use bluetooth to figure out closest supply that is open
        currentTask = new AquireRodTask(3, 1, driveTrain, rodGrabber, fieldController);
        break;

      case AQUIRE_NEW_ROD:
        //currentTask = new DropOffAtReactorTask();
        break;

      case DROP_OFF_AT_REACTOR:
        if (currentReactor == 0) {
          currentTask = new PickUpFromReactorTask(driveTrain, rodGrabber, fieldController);
          currentReactor = 1;
        } else {
          currentTask = new RobotTask();
        }
        break;

      default:
        //currentTask = new RobotTask();
        break;

    }
  }

  // Write to LCD
  float msPerFrame = (1000 / lcdFramesPerSecond); // 1000 * (1/FPS)
  if (currentTime > lastWriteTime + msPerFrame) {
    lcd.clear();

    if (rodGrabber->isAtSetpoint()) {
      lcd.print("At Setpoint!");
    } else {
      lcd.print("Moving to setpoint...");
      lcd.setCursor(0, 1);
      lcd.print(rodGrabber->readPotentiometer());
    }

    lastWriteTime = currentTime;
  }
}
