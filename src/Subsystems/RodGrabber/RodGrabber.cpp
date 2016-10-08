#include "RodGrabber.hpp"
/* constructor -
 * PIN_MOTOR - pin of the motor of the fourbar mechanism
 * PIN_GRABBER - pin of the servo motor of the graber
 * PIN_POTENTIOMETER - pin of the potentiometer of the fourbar mechanism
 */
RodGrabber::RodGrabber(int8 PIN_MOTOR, int8 PIN_GRABBER, int8 PIN_POTENTIOMETER) {
  _PIN_POTENTIOMETER = PIN_POTENTIOMETER;

  setpoint = 0.0;

  motor = new Motor(PIN_MOTOR);
  grabber.attach(PIN_GRABBER, 1000, 2000);
}

/* readPotentiometer - potentiometerValue
 * Reads the pot value
 * @returns Pot value from 0.0 to 1.0
 */
double RodGrabber::readPotentiometer() {
  // Return pot analog value (0-1023) devided by 1023
  return (double)analogRead(_PIN_POTENTIOMETER) / 1023.0;
}

/* moveUp -
 * Sets the motor setpoint to up
 */
void RodGrabber::moveUp() {
  setpoint = UP_SETPOINT;
}

/* moveDown -
 * Sets the motor setpoint to down
 */
void RodGrabber::moveDown() {
  setpoint = DOWN_SETPOINT;
}

/* grab -
 * Closes the grabber
 */
void RodGrabber::grab() {
  grabber.write(GRABBER_CLOSED_VALUE);
}

/* release -
 * Opens the grabber
 */
void RodGrabber::release() {
  grabber.write(GRABBER_OPEN_VALUE);
}

/* update -
 * Updates the PID loop for the motor
 */
void RodGrabber::update() {
  double error = setpoint - readPotentiometer();

  motor->write(error * kP);
}
