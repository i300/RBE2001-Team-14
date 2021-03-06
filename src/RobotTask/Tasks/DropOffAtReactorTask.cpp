#include "DropOffAtReactorTask.hpp"

DropOffAtReactorTask::DropOffAtReactorTask(int8 currentReactor, DriveTrain *driveTrain, RodGrabber *rodGrabber, FieldController *controller) : super(DROP_OFF_AT_REACTOR) {
  _driveTrain = driveTrain;
  _fieldController = controller;
  _rodGrabber = rodGrabber;

  _currentReactor = currentReactor;

  _driveTrain->resetLineCount();

  state = DR_DRIVE_TO_MAIN_LINE;
}

bool8 DropOffAtReactorTask::isFinished() {
  return (state == DR_FINISHED) && (millis() > timeLastStateSwitch + 500);
}

void DropOffAtReactorTask::update() {
  super::update();

  unsigned long currentTime = millis();

  switch (state) {
    case DR_DRIVE_TO_MAIN_LINE:
      /* Drive forward while line following until we see one perpendicular line */
      _driveTrain->followLine(0.2);
      if (_driveTrain->updateLineCount() == 1) {
        state = DR_TURN_TO_REACTOR;
        timeLastStateSwitch = currentTime;
      }
      break;

    case DR_TURN_TO_REACTOR:{
      /* Calculate which direction we need to turn based on the current reactor */
      int turnDirection = 0;
      if (_currentReactor == 0) {
        turnDirection = 1;
      } else {
        turnDirection = -1;
      }

      /* Drive forward so the turn center is on the middle of the line, then
       * turn a fixed distance (to move the line sensor off the line). Then turn
       * until we see the line we're turning onto and stop.
       */
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
          state = DR_DRIVE_TO_REACTOR;
        }
      }
      break;
    }

    case DR_DRIVE_TO_REACTOR:
      /* Drive forward while line following until the alignment switch is pressed.
       * Then stop moving and move the rodGrabber down
       */
      _driveTrain->followLine(_driveTrain->MAX_LINEFOLLOWING_SPEED);
      if (_driveTrain->isAlignmentSwitchPressed()) {
        _driveTrain->stop();
        _rodGrabber->moveDown();
        state = DR_DEPOSIT_ROD;
        timeLastStateSwitch = currentTime;
      }
      break;

    case DR_DEPOSIT_ROD:
      /* Wait until the rod grabber is at the setpoint, then release the grabber.
       * Then move the grabber back up.
       */
      if (_rodGrabber->isAtSetpoint()) {
        _rodGrabber->release();
        if (currentTime > timeLastStateSwitch + 500) {
          _rodGrabber->moveUp();
          if (_rodGrabber->isAtSetpoint()) {
            state = DR_TURN_AROUND;
            timeLastStateSwitch = currentTime;
          }
        }
      } else {
        _driveTrain->tankDrive(0.1, 0.1);
        timeLastStateSwitch = currentTime;
      }
      break;

    case DR_TURN_AROUND:
      /* Back up for 1 second, turn manually for 1 second, then turn onto
       * the line and move to the next state.
       */
      if (currentTime < timeLastStateSwitch + 1000) {
        _driveTrain->tankDrive(-0.2, -0.2);
      } else if (currentTime < timeLastStateSwitch + 2000) {
        _driveTrain->tankDrive(-0.2, 0.2);
      } else {
        if (_driveTrain->turnOntoLine(-0.2)) {
          _fieldController->radiationStatus = FieldController::RadiationStatus::kNoRadiation;
          state = DR_FINISHED;
          timeLastStateSwitch = currentTime;
        }
      }
      break;

    case DR_FINISHED:
      /* Re-align with the line for a small amount of time */
      _driveTrain->alignWithLine();
      break;
  }
}

int DropOffAtReactorTask::getState() {
  return state;
}

void DropOffAtReactorTask::finished() {

}
