#include "../../Utilities.h"
#include "../../Subsystems/DriveTrain/DriveTrain.hpp"
#include "../../Subsystems/RodGrabber/RodGrabber.hpp"
#include "../../FieldController/FieldController.hpp"
#include "../RobotTask.hpp"

#ifndef AQUIRE_ROD_TASK_H
#define AQUIRE_ROD_TASK_H

enum AquireRodState {
  BACK_UP,
  TURN_ONTO_LINE,
  DRIVE_TO_SUPPLY,
  FINISHED
};

class AquireRodTask : RobotTask {
private:
  typedef RobotTask super;

  AquireRodState state;

  DriveTrain *_driveTrain;
  FieldController *_fieldController;

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
