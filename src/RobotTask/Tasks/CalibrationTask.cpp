#include "CalibrationTask.hpp"

CalibrationTask::CalibrationTask(DriveTrain *driveTrain, RodGrabber *rodGrabber, FieldController *controller) : super(CALIBRATION) {
  _driveTrain = driveTrain;
  _fieldController = controller;
  _rodGrabber = rodGrabber;

  timeLastStateSwitch = millis();

  _driveTrain->resetLineCount();

  state = CS_SWING_LEFT;
}

bool8 CalibrationTask::isFinished() {
  return (state == CS_FINISHED) && (millis() > timeLastStateSwitch + 500);
}

void CalibrationTask::update() {
  super::update();

  unsigned long currentTime = millis();

  switch (state) {
    case CS_SWING_LEFT:
      _driveTrain->tankDrive(0.2, -0.2);
        _driveTrain->calibrateLineSensor();
      if (currentTime > timeLastStateSwitch + SWEEP_TIME) {
        state = CS_SWING_RIGHT;
        timeLastStateSwitch = currentTime;
      }
      break;

    case CS_SWING_RIGHT:
      _driveTrain->tankDrive(-0.2, 0.2);
        _driveTrain->calibrateLineSensor();
      if (currentTime > timeLastStateSwitch + SWEEP_TIME*2) {
        state = CS_SWING_FINAL;
        timeLastStateSwitch = currentTime;
      }
      break;

    case CS_SWING_FINAL:
      _driveTrain->tankDrive(0.2, -0.2);
        _driveTrain->calibrateLineSensor();
      if (currentTime > timeLastStateSwitch + SWEEP_TIME) {
        state = CS_FINISHED;
        timeLastStateSwitch = currentTime;
      }
      break;

    case CS_FINISHED:
      _driveTrain->alignWithLine();
      break;

  }
}

int CalibrationTask::getState() {
  return state;
}

void CalibrationTask::finished() {

}
