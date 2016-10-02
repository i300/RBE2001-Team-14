#include "../../Utilities.h"
#include "../../Motor/Motor.hpp"

#ifndef ROD_GRABBER_H
#define ROD_GRABBER_H

class RodGrabber {
private:
  Motor *motor;
  Servo grabber;

  int8 _PIN_POTENTIOMETER;
  double readPotentiometer();

  const double UP_SETPOINT = 0.42;
  const double DOWN_SETPOINT = 0.0;

  const int GRABBER_OPEN_VALUE = 0;
  const int GRABBER_CLOSED_VALUE = 180;

  double setpoint = 0;
  double kP = 5.0;
  double kI = 0.0;
  double kD = 0.0;

public:

  RodGrabber(int8 PIN_MOTOR, int8 PIN_GRABBER, int8 PIN_POTENTIOMETER);

  void moveUp();
  void moveDown();

  void grab();
  void release();

  void update();

};

#endif
