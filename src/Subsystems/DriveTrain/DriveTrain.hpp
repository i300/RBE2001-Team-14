#include <Arduino.h>
#include <QTRSensors.h>
#include <LiquidCrystal.h>
#include "../../Motor/Motor.hpp"
#include "../../Utilities.h"

/*
 * DriveTrain
 * Class to store the information of the drive train of the robot
 * such information includes:
 * leftMotor
 * rightMotor
 * lineSensors
 * 
 * See comments below for more information
 */

#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

enum DriveTrainInvertedSide { INVERTED_LEFT, INVERTED_RIGHT };

class DriveTrain {
  Motor *leftMotor, *rightMotor;
  QTRSensorsAnalog *_lineSensor;
  LiquidCrystal *_lcd;

  int _alignmentSwitchPin;

  // Line following constants
  const int LINEFOLLOW_CENTER_POSITION = 3500; // Measured
  const int TURN_ONTO_LINE_TOLERANCE = 250;
  const int MAX_TIME_BETWEEN_LINES = 500; // ms

  // Line following running variables
  unsigned long lastMeasurement = 0;
  double lastError = 0;
  double sumError = 0;
  unsigned long timeLastLineSeen = 0;
  int horizontalLinesSeen = 0;

  void writeToMotors(float left, float right);

public:
  DriveTrain(int leftMotorPin, int rightMotorPin, int alignmentSwitchPin,
             LiquidCrystal *lcd, QTRSensorsAnalog *lineSensor,
             DriveTrainInvertedSide inversion);
  void arcadeDrive(float speed, float rotation);
  void tankDrive(float left, float right);

  const double MAX_LINEFOLLOWING_SPEED = 0.275; // determined

  double followLine(float speed);
  bool8 turnOntoLine(float speed, int direction);
  int updateLineCount();
  void resetLineCount();

  bool8 isAlignmentSwitchPressed();

  void stop();

  // Line following constants
  double kP = 0.75;
  double kI = 0;
  double kD = 0.5;

  // Turning constants
  double kP_turn = 1.2;
};

#endif
