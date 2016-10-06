#include "Arduino.h"
#include "Utilities.h"
#include "RobotMap.h"

#include "FieldController/FieldController.hpp"
#include <QTRSensors.h>
#include <LiquidCrystal.h>

#include "Subsystems/DriveTrain/DriveTrain.hpp"
#include "Subsystems/RodGrabber/RodGrabber.hpp"

#include "RobotTask/RobotTask.hpp"
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

  lcd.print("Waiting to start...");

  pinMode(PIN_SENSOR_ALIGNMENT, INPUT);
  while (digitalRead(PIN_SENSOR_ALIGNMENT) == 1) {
    delay(10); // wait to start
  }

  lcd.clear();
  lcd.print("Calibrating...");

  uint8 sensorPins[] = { PIN_SENSOR_LINE1, PIN_SENSOR_LINE2, PIN_SENSOR_LINE3,
                         PIN_SENSOR_LINE4, PIN_SENSOR_LINE5, PIN_SENSOR_LINE6,
                         PIN_SENSOR_LINE7, PIN_SENSOR_LINE8 };
  lineSensor = new QTRSensorsAnalog(sensorPins, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, PIN_LINESENSOR_EMITTER);
  for (int i = 0; i < 40*5; i++) {  // make the calibration take about 10 seconds
    lineSensor->calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  }

  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(lineSensor->calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();

  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(lineSensor->calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();

  driveTrain = new DriveTrain(PIN_MOTOR_LEFT, PIN_MOTOR_RIGHT, PIN_SENSOR_ALIGNMENT,
                              &lcd, lineSensor, DriveTrainInvertedSide::INVERTED_LEFT);
  rodGrabber = new RodGrabber(PIN_MOTOR_GRABBER, PIN_SERVO_GRABBER, PIN_SENSOR_POT);

  rodGrabber->grab();

  currentTask = new RobotTask();
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
        currentTask = new StoreRodTask(3, driveTrain, rodGrabber, fieldController);
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

      default:
        //currentTask = RobotTask();
        break;

    }
  }

  // Write to LCD
  float msPerFrame = (1000 / lcdFramesPerSecond); // 1000 * (1/FPS)
  if (currentTime > lastWriteTime + msPerFrame) {
    lcd.clear();

    for (int i = 0; i < lineSensor->getNumSensors(); i++) {
      lcd.print(String(lineSensor->calibratedMaximumOn[i]) + " ");
      if (i == 3) lcd.setCursor(0, 1);
    }

    lastWriteTime = currentTime;
  }
}
