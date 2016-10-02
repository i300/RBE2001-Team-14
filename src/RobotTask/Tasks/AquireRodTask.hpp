#include "../../Utilities.h"
#include "../../Subsystems/DriveTrain/DriveTrain.hpp"
#include "../RobotTask.hpp"

#ifndef AQUIRE_ROD_TASK_H
#define AQUIRE_ROD_TASK_H

enum AquireRodState {
  BACK_UP,
  TURN_ONTO_LINE,
  DRIVE_TO_TARGET_LINE,
  DRIVE_TO_SUPPLY
};

class AquireRodTask : RobotTask {
private:
  typedef RobotTask super;

  AquireRodState state;

  DriveTrain *_driveTrain;
  int8 _currentLocation;
  int8 _rodLocation;

protected:
  void finished();

public:
  AquireRodTask(RobotLocation currentLocation, int8 rodLocation, DriveTrain *driveTrain);

  virtual bool8 isFinished();
  virtual void update();
};

#endif
