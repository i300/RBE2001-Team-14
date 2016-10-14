#include "../Utilities.h"
#include <Arduino.h>

/*
 * RobotTask
 * Base class that defines the robot's current "task"
 *
 */

#ifndef ROBOTTASK_H
#define ROBOTTASK_H

enum RobotTaskType {
  NO_TASK,
  CALIBRATION,
  AQUIRE_NEW_ROD,
  STORE_USED_ROD,
  PICKUP_FROM_REACTOR,
  DROP_OFF_AT_REACTOR
};

class RobotTask {

protected:
  RobotTaskType _type;

  bool8 _finished;

  unsigned long _timeStarted;

  virtual void finished();
  unsigned long getTimeStarted();

public:
  RobotTask();
  RobotTask(RobotTaskType type);

  RobotTaskType getType();
  virtual bool8 isFinished();
  virtual void update();

  virtual int getState();
};

#endif
