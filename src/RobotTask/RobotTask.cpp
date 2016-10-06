#include "RobotTask.hpp"

/* constructor -
 */
RobotTask::RobotTask() {
  _type = NO_TASK;
  _timeStarted = millis();
}

/* constructor -
 * type - the task type
 */
RobotTask::RobotTask(RobotTaskType type) {
  _type = type;
}

/* getTimeStarted - unsigned long timeStarted
 * Returns the millis() when the task was started.
 */
unsigned long RobotTask::getTimeStarted() {
  return _timeStarted;
}

/* getType - RobotTaskType type
 * Returns the type of the task
 */
RobotTaskType RobotTask::getType() {
  return _type;
}

/* isFinished - bool8 finished
 * Returns true if the task is finished.
 */
bool8 RobotTask::isFinished() {
  return true;
}

/* finished - void
 * Called once the task is finished. Cleans up the finished task.
 */
void RobotTask::finished() {

}

/* update - void
 * Updates the robot based on the current task. When the task is finished,
 * calls finished() once.
 */
void RobotTask::update() {
  if (isFinished() && !_finished) {
    finished();
    _finished = true;
  }
}

/* getState - int currentState
 * Returns the current state.
 */
int RobotTask::getState() {
  return 0;
}
