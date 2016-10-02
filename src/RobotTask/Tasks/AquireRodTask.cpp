#include "AquireRodTask.hpp"

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
      _driveTrain->arcadeDrive(-1, 0);

      unsigned long BACK_UP_TIME = 500; // ms
      if (getTimeStarted() > getTimeStarted() + BACK_UP_TIME) {
        state = TURN_ONTO_LINE;
      }

      break;
    }

    case TURN_ONTO_LINE: {
      _driveTrain->arcadeDrive(1, 1);



      break;
    }
  }
}

void AquireRodTask::finished() {

}
