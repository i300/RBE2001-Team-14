#include "Arduino.h"
#include "../../Motor/Motor.hpp"
#include "../../Utilities.h"

#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

enum DriveTrainInvertedSide { INVERTED_LEFT, INVERTED_RIGHT };

class DriveTrain {
  Motor *leftMotor, *rightMotor;

protected:
  void writeToMotors(float left, float right);

public:
  DriveTrain(int leftMotorPin, int rightMotorPin, DriveTrainInvertedSide inversion);
  void arcadeDrive(float speed, float rotation);
  void tankDrive(float left, float right);
  void stop();
};

#endif
