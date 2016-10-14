#include "../../Utilities.h"
#include "../../Motor/Motor.hpp"

/*
 * RodGrabber
 * Class to store the information of the rod Grabber.
 *
 * The rod grabber has information about the
 * motor - Motor for the fourbar slider crank mechanism
 * Servo - Servo in the grabber
 * Potentiometer - POT to know current angle of the fourbar
 */
#ifndef ROD_GRABBER_H
#define ROD_GRABBER_H

class RodGrabber {
private:
  Motor *motor;
  Servo grabber;

  int8 _PIN_POTENTIOMETER;

  //Variables for the position of the Fourbar mechanism
  const double UP_SETPOINT = 0.77;
  const double MID_SETPOINT = 0.61;
  const double DOWN_SETPOINT = 0.27;

  //Variables for the position of the grabber open/closed
  const int GRABBER_OPEN_VALUE = 0;
  const int GRABBER_CLOSED_VALUE = 180;

  // Tolerance for IsAtSetpoint
  const double SETPOINT_TOLERANCE = 0.02;

  //Variables for the PID loop
  double setpoint = 0; //desired position
  double kP = 7.5;

  double down_limit = 0.2;
  double up_limit = 1;

public:

  RodGrabber(int8 PIN_MOTOR, int8 PIN_GRABBER, int8 PIN_POTENTIOMETER);

  void stop();

  void moveUp();
  void moveDown();

  void grab();
  void release();

  void update();

  double readPotentiometer();

  bool8 isAtSetpoint();

};

#endif
