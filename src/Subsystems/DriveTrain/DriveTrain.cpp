#include "DriveTrain.hpp"

/* constructor
 *
 * leftMotorPin - PWM pin for left motor
 * rightMotorPin - PWM pin for right motor
 * inversion - Which side of the robot is inverted
 */
DriveTrain::DriveTrain(int leftMotorPin, int rightMotorPin, int alignmentSwitchPin,
                       LiquidCrystal *lcd, QTRSensorsAnalog *lineSensor,
                       DriveTrainInvertedSide inversion) {
  _lineSensor = lineSensor;
  _lcd = lcd;
  _alignmentSwitchPin = alignmentSwitchPin;

  pinMode(_alignmentSwitchPin, INPUT_PULLUP);

  switch (inversion) {
    case INVERTED_LEFT:
      leftMotor = new Motor(leftMotorPin, true);
      rightMotor = new Motor(rightMotorPin, false);
      break;

    case INVERTED_RIGHT:
      leftMotor = new Motor(leftMotorPin, false);
      rightMotor = new Motor(rightMotorPin, true);
      break;

    default:
      break;
  }
}

/* writeToMotors - void
 *
 * left - Left motor speed
 * right - Light motor speed
 */
void DriveTrain::writeToMotors(float left, float right) {
  leftMotor->write(left);
  rightMotor->write(right);
}

/* arcadeDrive - void
 * Drives robot based on a forward speed and rotation value
 *
 * speed - Speed robot will drive at
 * rotation - Value from -1 (left) to 1 (right)
 */
void DriveTrain::arcadeDrive(float speed, float rotation) {
  if (rotation == 0) tankDrive(speed, speed);

  // constrain speed and roation to intended values (-1 to 1)
  speed = constrain(speed, -1, 1);
  rotation = constrain(rotation, -1, 1);

  float leftMotorSpeed, rightMotorSpeed;

  // credit to Joel Gallant
  // https://gist.github.com/joelg236/a294a03a1094167ff49f
  if (speed > 0.0) {
    if (rotation > 0.0) {
      leftMotorSpeed = speed - rotation;
      rightMotorSpeed = max(speed, rotation);
    } else {
      leftMotorSpeed = max(speed, -rotation);
      rightMotorSpeed = speed + rotation;
    }
  } else {
    if (rotation > 0.0) {
      leftMotorSpeed = -max(-speed, rotation);
      rightMotorSpeed = speed + rotation;
    } else {
      leftMotorSpeed = speed - rotation;
      rightMotorSpeed = -max(-speed, -rotation);
    }
  }

  // drive robot
  writeToMotors(leftMotorSpeed, rightMotorSpeed);
}

/* tankDrive - void
 * Drive robot based on left values and right speeds
 *
 * left - Left speed
 * right - Right speed
 */
void DriveTrain::tankDrive(float left, float right) {
  // constrain speeds to intended values (-1 to 1)
  left = constrain(left, -1, 1);
  right = constrain(right, -1, 1);

  // write to motors
  writeToMotors(left, right);
}

/*
 *
 */
void DriveTrain::calibrateLineSensor() {
  _lineSensor->calibrate();
}

/* followLine - double
 * Drive the robot by following a line using PID control
 *
 * speed - speed to drive forward
 * @returns PID loop result
 */
double DriveTrain::followLine(float speed) {
  unsigned long currentTime = millis();

  unsigned int sensorValues[_lineSensor->getNumSensors()];
  unsigned int linePosition = _lineSensor->readLine(sensorValues);

  // Reset sum of error and last error if measurements are very out of date
  if (currentTime > lastMeasurement + 1000) {
    sumError = 0;
    lastError = 0;
  }

  double error = ((int)linePosition - LINEFOLLOW_CENTER_POSITION) / (double)LINEFOLLOW_CENTER_POSITION;
  double iPart = sumError;
  double dPart = error - lastError;

  sumError += error;
  lastError = error;

  float rotation = (error * kP) + (iPart * kI) + (dPart * kD);

  arcadeDrive(speed, rotation);

  lastMeasurement = currentTime;

  return rotation;
}

/* alignWithLine - Drive the robot by looking for a line using PID control
 *
 * speed - speed to drive forward
 * @returns PID loop result
 */
double DriveTrain::alignWithLine() {
  unsigned long currentTime = millis();

  unsigned int sensorValues[_lineSensor->getNumSensors()];
  unsigned int linePosition = _lineSensor->readLine(sensorValues);

  // Reset sum of error and last error if measurements are very out of date
  if (currentTime > lastMeasurement + 1000) {
    sumError = 0;
    lastError = 0;
  }

  double error = ((int)linePosition - LINEFOLLOW_CENTER_POSITION) / (double)LINEFOLLOW_CENTER_POSITION;
  double iPart = sumError;
  double dPart = error - lastError;

  sumError += error;
  lastError = error;

  float rotation = (error * kP_align) + (iPart * kI_align) + (dPart * kD_align);

  tankDrive(-rotation, rotation);

  lastMeasurement = currentTime;

  return rotation;
}

/* turnOntoLine - bool8
 * Turns the robot towards a new line
 *
 * speed - speed to drive
 * direction - direction of turning
 * @returns true or false depending if the robot achieved the task
 */
bool8 DriveTrain::turnOntoLine(float speed) {
  speed = constrain(speed, -1.0, 1.0);

  unsigned int sensorValues[_lineSensor->getNumSensors()];
  _lineSensor->readCalibrated(sensorValues);
  unsigned int middleSensorValue = sensorValues[6];

  int error = 1000 - middleSensorValue;

  Serial.println("Error: " + String(error));

  if (abs(error) < TURN_ONTO_LINE_TOLERANCE) {
    stop();
    return true;
  } else {
    speed = speed * kP_turn * (error / 1000.0);

    tankDrive(speed, -speed);
    return false;
  }
}

/* updateLineCount - int
 * keeps track of how many horizontal lines the robot has traveled
 *
 * @returns the number of horizontal lines traveled
 */
int DriveTrain::updateLineCount() {
  unsigned char numSensors = _lineSensor->getNumSensors();
  unsigned int sensorValues[numSensors];
  _lineSensor->readCalibrated(sensorValues);

  unsigned long currentTime = millis();

  if (sensorValues[0] > 900 && sensorValues[numSensors-1] > 900) {
    if (currentTime > timeLastLineSeen + MAX_TIME_BETWEEN_LINES) {
      horizontalLinesSeen += 1;
      timeLastLineSeen = currentTime;
    }
  }

  return horizontalLinesSeen;
}

/* resetLineCount - void
 * resets the number of horizontal lines the robot has already traveled
 */
void DriveTrain::resetLineCount() {
  horizontalLinesSeen = 0;
}

/* isAlignmentSwitchPressed - bool8
 * @returns 0 or 1 depending if the switch has been pressed
 */
bool8 DriveTrain::isAlignmentSwitchPressed() {
  Serial.println(digitalRead(_alignmentSwitchPin));
  return !digitalRead(_alignmentSwitchPin);
}

/* stop - void
 * Sends zero to the drive motors to stop motion
 */
void DriveTrain::stop() {
  writeToMotors(0, 0);
}
