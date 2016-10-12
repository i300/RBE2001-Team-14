#include "PickUpFromReactorTask.hpp"

PickUpFromReactorTask::PickUpFromReactorTask(DriveTrain *driveTrain, RodGrabber *rodGrabber, FieldController *controller) : super(PICKUP_FROM_REACTOR) {
  _driveTrain = driveTrain;
  _fieldController = controller;
  _rodGrabber = rodGrabber;

  _driveTrain->resetLineCount();

  state = PFR_DRIVE_FORWARD;
}

bool8 PickUpFromReactorTask::isFinished() {
  return (state == PFR_FINISHED) && (millis() > timeLastStateSwitch + 500);
}

void PickUpFromReactorTask::update() {
  super::update();

  unsigned long currentTime = millis();

  switch (state) {
    case PFR_DRIVE_FORWARD:
      /* Follow the line and ensure the arm is in the up position and the
       * grabber is open. When the switch is pressed, move the arm down and
       * continue to the next state
       */
      _driveTrain->followLine(_driveTrain->MAX_LINEFOLLOWING_SPEED);
      _rodGrabber->moveUp();
      _rodGrabber->release();
      if (_driveTrain->isAlignmentSwitchPressed()) {
        _driveTrain->stop();
        _rodGrabber->moveDown();
        state = PFR_PICK_UP;
        timeLastStateSwitch = currentTime;
      }
      break;
    case PFR_PICK_UP:
      /* Wait until the arm is in the down position, then grab the rod and
       * move the arm up. Once the arm is up, move to the next state.
       */
      if (_rodGrabber->isAtSetpoint()) {
        _rodGrabber->grab();
        if (currentTime > timeLastStateSwitch + 500) {
          _rodGrabber->moveUp();
          if (_rodGrabber->isAtSetpoint()) {
            state = PFR_TURN_AROUND;
            _fieldController->radiationStatus = FieldController::RadiationStatus::kLowRadiation;
            timeLastStateSwitch = currentTime;
          }
        }
      } else {
        timeLastStateSwitch = currentTime;
      }
      break;
    case PFR_TURN_AROUND:
      /* Back up for 1 second, turn manually for 1 second, then turn onto
       * the line and move to the next state.
       */
      if (currentTime < timeLastStateSwitch + 1000) {
        _driveTrain->tankDrive(-0.2, -0.2);
      } else if (currentTime < timeLastStateSwitch + 2000) {
        _driveTrain->tankDrive(-0.2, 0.2);
      } else {
        if (_driveTrain->turnOntoLine(-0.2)) {
          state = PFR_FINISHED;
          timeLastStateSwitch = currentTime;
        }
      }
      break;
    case PFR_FINISHED:
      /* Re-align with the line for a small amount of time */
      _driveTrain->alignWithLine();
      break;
  }
}

int PickUpFromReactorTask::getState() {
  return state;
}

void PickUpFromReactorTask::finished() {

}
