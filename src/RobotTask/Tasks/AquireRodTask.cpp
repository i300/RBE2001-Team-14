#include "AquireRodTask.hpp"

/* constructor -
 * rodLocation - the current location of the rod
 * driveTrain - a pointer to the drive train
*/
AquireRodTask::AquireRodTask(RobotLocation currentLocation, int8 rodLocation, DriveTrain *driveTrain) : super() {
  _rodLocation = rodLocation;
  _driveTrain = driveTrain;

  state = BACK_UP;
}

/* isFinished - bool8 finished
 * returns ____ when the task is finished 
 */
bool8 AquireRodTask::isFinished() {
  return false;
}


/* update - void
 * Updates the robot based on the current task. 
 */
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

/* finished - void 
 * Called once the task is finished. Cleans up the finished task
 */
void AquireRodTask::finished() {

}
