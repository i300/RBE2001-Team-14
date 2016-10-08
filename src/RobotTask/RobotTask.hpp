#include "../Utilities.h"
#include <Arduino.h>

/*
 * RobotTask
 * Class to store the current "task" of the RobotTask
 *
 * A task has a type and data associated with it. Data is defined differently
 * for every type. See comment below for more information on data.
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
