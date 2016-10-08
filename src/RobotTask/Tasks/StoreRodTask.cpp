#include "StoreRodTask.hpp"

StoreRodTask::StoreRodTask(int8 rodLocation, DriveTrain *driveTrain, RodGrabber *rodGrabber, FieldController *controller) : super() {
  _driveTrain = driveTrain;
  _fieldController = controller;
  _rodGrabber = rodGrabber;
  _rodLocation = rodLocation;

  _driveTrain->resetLineCount();

  state = SRR_DRIVE_TO_LINE;
}

bool8 StoreRodTask::isFinished() {
  return false;//state == SRR_FINISHED;
}

void StoreRodTask::update() {
  super::update();

  unsigned long currentTime = millis();

  switch (state) {
    case SRR_DRIVE_TO_LINE: {

      _rodGrabber->moveUp();

      _driveTrain->followLine(_driveTrain->MAX_LINEFOLLOWING_SPEED);
      int lineCount = _driveTrain->updateLineCount();

      if (lineCount == _rodLocation) {
        state = SRR_TURN_ONTO_LINE;
        timeLastStateSwitch = currentTime;
      }

      break;
    }

    case SRR_TURN_ONTO_LINE:
      if (currentTime < timeLastStateSwitch + 200) {
        _driveTrain->arcadeDrive(0.225, 0);
      } else if (currentTime < timeLastStateSwitch + 600) {
        _driveTrain->tankDrive(-_driveTrain->MAX_LINEFOLLOWING_SPEED, _driveTrain->MAX_LINEFOLLOWING_SPEED);
      } else {
        if (_driveTrain->turnOntoLine(-0.2)) {
          _driveTrain->stop();
          timeLastStateSwitch = currentTime;
          state = SRR_ALIGN;
        }
      }

      break;

    case SRR_ALIGN:
      if (currentTime < timeLastStateSwitch + 500) {
        _driveTrain->alignWithLine();
      } else {
        state = SRR_DRIVE_TO_STORAGE;
        timeLastStateSwitch = currentTime;
      }
      break;


    case SRR_DRIVE_TO_STORAGE:
      _driveTrain->followLine(0.225);
      if (_driveTrain->isAlignmentSwitchPressed()) {
        _driveTrain->stop();
        state = SRR_INSERT_ROD;
        timeLastStateSwitch = currentTime;
      }
      break;

    case SRR_INSERT_ROD:
      _rodGrabber->release();
      if (currentTime > timeLastStateSwitch + 250) {
        state = SRR_TURN_AROUND;
        timeLastStateSwitch = currentTime;
      }
      break;

    case SRR_TURN_AROUND:
      if (currentTime < timeLastStateSwitch + 500) { // Back up
        _driveTrain->arcadeDrive(-_driveTrain->MAX_LINEFOLLOWING_SPEED, 0);
      } else if (currentTime < timeLastStateSwitch + 2125) { // Turn for time
        _driveTrain->tankDrive(-0.2, 0.2);
      } else { // Turn until sensor
        if (_driveTrain->turnOntoLine(-0.2)) {
          _driveTrain->stop();
          state = SRR_FINISHED;
        }
      }
      break;

    case SRR_FINISHED:
      _driveTrain->alignWithLine();
      break;

    default:
      break;
  }
}

int StoreRodTask::getState() {
  return state;
}

void StoreRodTask::finished() {

}
