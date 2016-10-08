#include "StoreRodTask.hpp"
/* constructor - 
 * rodLocation - the current location of the rod
 * *driveTrain - a pointer to the drive train
 * *rodGrabber - a pointer to the rod grabber
 * *controller - a pointer to the controller of the field
 */
StoreRodTask::StoreRodTask(int8 rodLocation, DriveTrain *driveTrain, RodGrabber *rodGrabber, FieldController *controller) : super() {
  _driveTrain = driveTrain;
  _fieldController = controller;
  _rodGrabber = rodGrabber;
  _rodLocation = rodLocation;

  _driveTrain->resetLineCount();

  state = SRR_DRIVE_TO_LINE;
}

/* isFinished - bool8 finished
 * returns _____ when the task is finished
 */
bool8 StoreRodTask::isFinished() {
  return false;//state == SRR_FINISHED;
}

/* update - void 
 * Updates the robot based on the current task.  
 */
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
      if (currentTime < timeLastStateSwitch + 250) {
        _driveTrain->arcadeDrive(_driveTrain->MAX_LINEFOLLOWING_SPEED, 0);
      } else if (currentTime < timeLastStateSwitch + 750) {
        _driveTrain->tankDrive(-_driveTrain->MAX_LINEFOLLOWING_SPEED, _driveTrain->MAX_LINEFOLLOWING_SPEED);
      } else {
        if (_driveTrain->turnOntoLine(0.2, -1)) {
          _driveTrain->stop();
          timeLastStateSwitch = currentTime;
          state = SRR_REALIGN_WITH_LINE;
        }
      }

      break;

    case SRR_REALIGN_WITH_LINE:
      _driveTrain->followLine(0.1);
      if (millis() > timeLastStateSwitch + 1000) {
        state = SRR_DRIVE_TO_STORAGE;
      }
      break;

    case SRR_DRIVE_TO_STORAGE:
      _driveTrain->followLine(_driveTrain->MAX_LINEFOLLOWING_SPEED-0.05);
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
      if (currentTime < timeLastStateSwitch + 500) {
        _driveTrain->arcadeDrive(-_driveTrain->MAX_LINEFOLLOWING_SPEED, 0);
      } else if (currentTime < timeLastStateSwitch + 1000) {
        _driveTrain->tankDrive(-_driveTrain->MAX_LINEFOLLOWING_SPEED, _driveTrain->MAX_LINEFOLLOWING_SPEED);
      } else {
        if (_driveTrain->turnOntoLine(-0.2, 1)) {
          _driveTrain->stop();
          state = SRR_FINISHED;
        }
      }
      break;

    case SRR_FINISHED:
      break;

    default:
      break;
  }
}
/* getState - int currentState
 * Returns the current state. */
int StoreRodTask::getState() {
  return state;
}
/* finished - void 
 * Called once the task is finished. Cleans up the finished task
 */
void StoreRodTask::finished() {

}
