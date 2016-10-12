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
#include "RobotTask/Tasks/DropOffAtReactorTask.hpp"

// Sensors and controllers
FieldController *fieldController;
QTRSensorsAnalog *lineSensor;

// Subsystems
DriveTrain *driveTrain;
RodGrabber *rodGrabber;

// Task stuff
RobotTask *currentTask;
int8 currentReactor = 0; // 0 = Reactor A, 1 = Reactor B
int8 lastLocation = 0;

// LCD
LiquidCrystal lcd(40, 41, 42, 43, 44, 45);
unsigned long lastWriteTime = 0;
const float lcdFramesPerSecond = 30;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");

  fieldController = new FieldController(ROBOT_ID);
  fieldController->setup();

  lcd.begin(16, 2); // Begin LCD
  delay(50); // Hold up, wait a minute
  lcd.clear(); // Clear LCD

  delay(500);

  lcd.print("Waiting to start...");

  pinMode(PIN_SENSOR_ALIGNMENT, INPUT_PULLUP);
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
  rodGrabber->release();

  lcd.print("Waiting 4 field...");

  // Wait to start task loop until the field is sending messages
  while (!fieldController->hasRecievedMessage()) {
    fieldController->update();
  }

  currentTask = new CalibrationTask(driveTrain, rodGrabber, fieldController);
}

long nextTime = millis() + 2500;
bool8 grabbed = false;

void loop() {
  unsigned long currentTime = millis();

  // Update bluetooth controller
  fieldController->update();
  //fieldController->printStatus();

  // Stop all actions if robot is stopped
  if (fieldController->getStopped()) {
    driveTrain->stop();
    rodGrabber->stop();
    return;
  }

  // Update Subsystems
  rodGrabber->update();

  // Update current task
  currentTask->update();

  // Update state machine if task is finished
  RobotTaskType taskType = currentTask->getType();
  if (currentTask->isFinished()) {
    // Delete old task off the heap
    delete currentTask;

    switch (taskType) {

      case NO_TASK:
        // Nothing to see here...
        break;

      case CALIBRATION:
        currentTask = new PickUpFromReactorTask(driveTrain, rodGrabber, fieldController);
        //currentTask = new AquireRodTask(3, 2, driveTrain, rodGrabber, fieldController);
        break;

      case PICKUP_FROM_REACTOR: {
        // TODO: Use bluetooth to figure out closest storage that is open
        int8 closestOpenStorage = fieldController->getClosestOpenStorage(currentReactor);
        currentTask = new StoreRodTask(closestOpenStorage, currentReactor, driveTrain, rodGrabber, fieldController);
        lastLocation = closestOpenStorage;
        break;
      }

      case STORE_USED_ROD: {
        // TODO: Use bluetooth to figure out closest supply that is open
        int8 closestFullSupply = fieldController->getClosestFullSupply(currentReactor);
        currentTask = new AquireRodTask(lastLocation, closestFullSupply, currentReactor, driveTrain, rodGrabber, fieldController);
        lastLocation = closestFullSupply;
        break;
      }

      case AQUIRE_NEW_ROD:
        currentTask = new DropOffAtReactorTask(currentReactor, driveTrain, rodGrabber, fieldController);
        break;

      case DROP_OFF_AT_REACTOR:
        if (currentReactor == 0) {
          currentReactor = 1;
          currentTask = new PickUpFromReactorTask(driveTrain, rodGrabber, fieldController);
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

    #ifndef DEBUG
      if (fieldController->radiationStatus == FieldController::RadiationStatus::kLowRadiation) {
        lcd.write("Low Radiation!");
      } else if (fieldController->radiationStatus == FieldController::RadiationStatus::kHighRadiation) {
        lcd.write("HIGH RADIATION!!");
      } else {
        lcd.write("No Radiation.");
      }
    #else
      if (rodGrabber->isAtSetpoint()) {
        lcd.print("At Setpoint!");
      } else {
        lcd.print("Moving to setpoint...");
      }
      lcd.setCursor(0, 1);
      lcd.print(rodGrabber->readPotentiometer());
    #endif

    lastWriteTime = currentTime;
  }
}
