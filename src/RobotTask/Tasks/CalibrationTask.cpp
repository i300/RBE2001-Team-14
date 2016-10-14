#include "CalibrationTask.hpp"

CalibrationTask::CalibrationTask(DriveTrain *driveTrain, RodGrabber *rodGrabber, FieldController *controller) : super(CALIBRATION) {
  _driveTrain = driveTrain;
  _fieldController = controller;
  _rodGrabber = rodGrabber;

  timeLastStateSwitch = millis();

  _driveTrain->resetLineCount();

  state = CS_SWING_LEFT;
}

/* isFinished - bool8
 * returns true when the task is finished
 */
bool8 CalibrationTask::isFinished() {
  return (state == CS_FINISHED) && (millis() > timeLastStateSwitch + 500);
}

/* update - void
 * Updates the task
 */
void CalibrationTask::update() {
  super::update();

  unsigned long currentTime = millis();

  switch (state) {
    case CS_SWING_LEFT:
      /* Have the robot turn about its center left while calibrating the line sensor */
      _driveTrain->tankDrive(0.2, -0.2);
        _driveTrain->calibrateLineSensor();
      if (currentTime > timeLastStateSwitch + SWEEP_TIME) {
        state = CS_SWING_RIGHT;
        timeLastStateSwitch = currentTime;
      }
      break;

    case CS_SWING_RIGHT:
      /* Have the robot turn about its center right while calibrating the line sensor */
      _driveTrain->tankDrive(-0.2, 0.2);
      _driveTrain->calibrateLineSensor();
      if (currentTime > timeLastStateSwitch + SWEEP_TIME*2) {
        state = CS_SWING_FINAL;
        timeLastStateSwitch = currentTime;
      }
      break;

    case CS_SWING_FINAL:
      /* Have the robot turn about its center left while calibrating the line sensor */
      _driveTrain->tankDrive(0.2, -0.2);
      _driveTrain->calibrateLineSensor();
      if (currentTime > timeLastStateSwitch + SWEEP_TIME) {
        state = CS_FINISHED;
        timeLastStateSwitch = currentTime;
      }
      break;

    case CS_FINISHED:
      /* Re-align with the line to make sure we didn't overshoot the line. */
      _driveTrain->alignWithLine();
      break;

  }
}

/* getState - int
 * Returns the current state. Used for debugging.
 */
int CalibrationTask::getState() {
  return state;
}

/* finished - void
 * Called once the task is finished. Cleans up the finished task
 */
void CalibrationTask::finished() {

}
