#include "DriveTrain.hpp"

/* constructor
 *
 * leftMotorPin - PWM pin for left motor
 * rightMotorPin - PWM pin for right motor
 * inversion - Which side of the robot is inverted
 */
DriveTrain::DriveTrain(int leftMotorPin, int rightMotorPin, DriveTrainInvertedSide inversion) {
  leftMotor = new Motor(leftMotorPin);
  rightMotor = new Motor(rightMotorPin);

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

/* writeToMotors
 *
 * left - Left motor speed
 * right - Light motor speed
 */
void DriveTrain::writeToMotors(float left, float right) {
  leftMotor->write(left);
  rightMotor->write(right);
}

/* arcadeDrive - Drives robot based on a forward speed and rotation value
 *
 * speed - Speed robot will drive at
 * rotation - Value from -1 (left) to 1 (right)
 */
void DriveTrain::arcadeDrive(float speed, float rotation) {
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

/* tankDrive - Drive robot based on left values and right speeds
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

/* stop - Sends zero to the drive motors
 *
 */
void DriveTrain::stop() {
  writeToMotors(0, 0);
}
