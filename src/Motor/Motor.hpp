#include "Arduino.h"
#include <Servo.h>
#include "../Utilities.h"

#ifndef MOTOR_H
#define MOTOR_H

class Motor {
  Servo motor;

  bool16 inverted;

  static const int MOTOR_MAX_VALUE = 180;
  static const int MOTOR_MIN_VALUE = 0;

protected:
  int speedToMotorValue(float speed);

public:
  Motor(int8 pin);
  Motor(int8 pin, bool16 inverted);
  void write(float speed);

};

#endif
