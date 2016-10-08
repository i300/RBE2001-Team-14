#include "AquireRodTask.hpp"

/* constructor -
 * _rodLocation - the current location of the rod
 * _driveTrain - 
*/
AquireRodTask::AquireRodTask(RobotLocation currentLocation, int8 rodLocation, DriveTrain *driveTrain) : super() {
  _rodLocation = rodLocation;
  _driveTrain = driveTrain;

  state = BACK_UP;
}

bool8 AquireRodTask::isFinished() {
  return false;
}

void AquireRodTask::update() {
  super::update();

  switch (state) {
    case BACK_UP: {

      break;
    }

    case TURN_ONTO_LINE: {


      break;
    }

    case DRIVE_TO_SUPPLY: {

      break;

    }

    case FINISHED: {

      break;
    }
  }
}

void AquireRodTask::finished() {

}
