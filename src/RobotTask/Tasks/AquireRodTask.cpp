#include "AquireRodTask.hpp"

/* constructor -
 * rodLocation - the current location of the rod
 * *driveTrain - a pointer to the drive train
*/
AquireRodTask::AquireRodTask(int8 currentLocation, int8 rodLocation, int8 currentReactor,
                             DriveTrain *driveTrain, RodGrabber *rodGrabber,
                             FieldController *controller) : super(AQUIRE_NEW_ROD) {
  _driveTrain = driveTrain;
  _rodGrabber = rodGrabber;
  _fieldController = controller;
  _currentReactor = currentReactor;

  _driveTrain->resetLineCount();

  _initialLocation = currentLocation;
  _rodLocation = rodLocation;

  if (_initialLocation == _rodLocation) {
    state = AR_DRIVE_DIRECT_TO_SUPPLY;
  } else {
    state = AR_DRIVE_TO_MAIN_LINE;
  }
}

/* isFinished - bool8
 * returns true when the task is finished
 */
bool8 AquireRodTask::isFinished() {
  return (state == AR_FINISHED) && (millis() > timeLastStateSwitch + 500);
}


/* update - void
 * Updates the robot based on the current task.
 */
void AquireRodTask::update() {
  super::update();

  unsigned long currentTime = millis();

  switch (state) {
    case AR_DRIVE_TO_MAIN_LINE:
      _driveTrain->followLine(0.2);
      if (_driveTrain->updateLineCount() == 1) {
        state = AR_TURN_ONTO_MAIN_LINE;
        timeLastStateSwitch = currentTime;
      }
      break;

    case AR_TURN_ONTO_MAIN_LINE: {
      int turnDirection = 0;
      if (_rodLocation > _initialLocation) {
        turnDirection = 1;
      } else {
        turnDirection = -1;
      }

      if (currentTime < timeLastStateSwitch + 750) {
        _driveTrain->arcadeDrive(0.225, 0);
      } else if (currentTime < timeLastStateSwitch + 1150) {
        _driveTrain->tankDrive(-_driveTrain->MAX_LINEFOLLOWING_SPEED * turnDirection,
                               _driveTrain->MAX_LINEFOLLOWING_SPEED * turnDirection);
      } else {
        if (_driveTrain->turnOntoLine(-0.2 * turnDirection)) {
          _driveTrain->stop();
          _driveTrain->resetLineCount();
          timeLastStateSwitch = currentTime;
          state = AR_DRIVE_TO_SUPPLY;
        }
      }
      break;
    }

    case AR_DRIVE_TO_SUPPLY: {
      _driveTrain->followLine(0.2);
      if (_driveTrain->updateLineCount() == abs(_rodLocation - _initialLocation)) {
        state = AR_TURN_TO_SUPPLY;
        timeLastStateSwitch = currentTime;
      }
      break;
    }

    case AR_TURN_TO_SUPPLY: {
      int turnDirection = 0;
      if (_rodLocation > _initialLocation) {
        turnDirection = -1;
      } else {
        turnDirection = 1;
      }

      if (currentTime < timeLastStateSwitch + 625) {
        _driveTrain->arcadeDrive(0.225, 0);
      } else if (currentTime < timeLastStateSwitch + 1025) {
        _driveTrain->tankDrive(-_driveTrain->MAX_LINEFOLLOWING_SPEED * turnDirection,
                               _driveTrain->MAX_LINEFOLLOWING_SPEED * turnDirection);
      } else {
        if (_driveTrain->turnOntoLine(-0.2 * turnDirection)) {
          _driveTrain->stop();
          _driveTrain->resetLineCount();
          timeLastStateSwitch = currentTime;
          state = AR_ALIGN_WITH_LINE;
        }
      }
      break;
    }

    case AR_ALIGN_WITH_LINE:
      if (currentTime < timeLastStateSwitch + 500) {
        _driveTrain->alignWithLine();
      } else {
        state = AR_DRIVE_DIRECT_TO_SUPPLY;
        timeLastStateSwitch = currentTime;
      }
      break;


    case AR_DRIVE_DIRECT_TO_SUPPLY:
      _driveTrain->followLine(0.225);
      if (_driveTrain->isAlignmentSwitchPressed()) {
        state = AR_PICKUP_ROD;
        timeLastStateSwitch = currentTime;
      }
      break;

    case AR_PICKUP_ROD:
      /* Wait 1/2 a second to grab the rod, then switch state */

      _driveTrain->tankDrive(0.1, 0.1);
      if (currentTime < timeLastStateSwitch + 500) {

      } else if (currentTime < timeLastStateSwitch + 1000) {
        _rodGrabber->grab();
      } else if (currentTime < timeLastStateSwitch + 1500) {
        _fieldController->radiationStatus = FieldController::RadiationStatus::kHighRadiation;
        state = AR_TURN_AROUND;
        timeLastStateSwitch = currentTime;
      }
      break;

    case AR_TURN_AROUND:
      if (currentTime < timeLastStateSwitch + 500) { // Back up
        _driveTrain->arcadeDrive(-_driveTrain->MAX_LINEFOLLOWING_SPEED, 0);
      } else if (currentTime < timeLastStateSwitch + 2125) { // Turn for time
        _driveTrain->tankDrive(-0.2, 0.2);
      } else { // Turn until sensor
        if (_driveTrain->turnOntoLine(-0.2)) {
          _driveTrain->stop();
          state = AR_FINISHED;
          timeLastStateSwitch = currentTime;
        }
      }
      break;

    case AR_FINISHED:
      _driveTrain->alignWithLine();
      break;
  }
}

/* finished - void
 * Called once the task is finished. Cleans up the finished task
 */
void AquireRodTask::finished() {

}
