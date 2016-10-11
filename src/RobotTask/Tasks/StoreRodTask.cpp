#include "StoreRodTask.hpp"
/* constructor -
 * rodLocation - the current location of the rod
 * *driveTrain - a pointer to the drive train
 * *rodGrabber - a pointer to the rod grabber
 * *controller - a pointer to the controller of the field
 */
StoreRodTask::StoreRodTask(int8 rodLocation, int8 currentReactor, DriveTrain *driveTrain, RodGrabber *rodGrabber, FieldController *controller) : super(STORE_USED_ROD) {
  _driveTrain = driveTrain;
  _fieldController = controller;
  _rodGrabber = rodGrabber;
  _rodLocation = rodLocation;
  _currentReactor = currentReactor;

  _driveTrain->resetLineCount();

  state = SRR_DRIVE_TO_LINE;
}

/* isFinished - bool8 finished
 * returns _____ when the task is finished
 */
bool8 StoreRodTask::isFinished() {
  return (state == SRR_FINISHED) && (millis() > timeLastStateSwitch + 1000);
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
      int rodLocation = 0;

      // calculate # of lines to drive based on the current reactor we're coming from
      if (_currentReactor == 0)
        rodLocation = _rodLocation;
      else
        // total # of supplys+1 - rodLocation relative to reactor A
        rodLocation = 5 - _rodLocation;

      if (lineCount == rodLocation) {
        state = SRR_TURN_ONTO_LINE;
        timeLastStateSwitch = currentTime;
      }

      break;
    }

    case SRR_TURN_ONTO_LINE: {
      int turnDirection = 1;
      if (_currentReactor == 1) {
        turnDirection = -1;
      }
      if (currentTime < timeLastStateSwitch + 200) {
        _driveTrain->arcadeDrive(0.225, 0);
      } else if (currentTime < timeLastStateSwitch + 600) {
        _driveTrain->tankDrive(-_driveTrain->MAX_LINEFOLLOWING_SPEED * turnDirection,
                               _driveTrain->MAX_LINEFOLLOWING_SPEED * turnDirection);
      } else {
        if (_driveTrain->turnOntoLine(-0.2 * turnDirection)) {
          _driveTrain->stop();
          timeLastStateSwitch = currentTime;
          state = SRR_ALIGN;
        }
      }

      break;
    }

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
          timeLastStateSwitch = currentTime;
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
