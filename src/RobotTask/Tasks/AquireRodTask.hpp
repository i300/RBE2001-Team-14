#include "../../Utilities.h"
#include "../../Subsystems/DriveTrain/DriveTrain.hpp"
#include "../../Subsystems/RodGrabber/RodGrabber.hpp"
#include "../../FieldController/FieldController.hpp"
#include "../RobotTask.hpp"

#ifndef AQUIRE_ROD_TASK_H
#define AQUIRE_ROD_TASK_H

enum AquireRodState {
  AR_DRIVE_TO_MAIN_LINE,
  AR_TURN_ONTO_MAIN_LINE,
  AR_DRIVE_TO_SUPPLY,
  AR_TURN_TO_SUPPLY,
  AR_ALIGN_WITH_LINE,
  AR_DRIVE_DIRECT_TO_SUPPLY,
  AR_PICKUP_ROD,
  AR_TURN_AROUND,
  AR_FINISHED
};

class AquireRodTask : public RobotTask {
private:
  typedef RobotTask super;

  AquireRodState state;
  unsigned long timeLastStateSwitch = 0;

  DriveTrain *_driveTrain;
  RodGrabber *_rodGrabber;
  FieldController *_fieldController;

  int8 _initialLocation;
  int8 _rodLocation;
  int8 _currentReactor;

protected:
  void finished();

public:
  AquireRodTask(int8 currentLocation, int8 rodLocation, int8 currentReactor, DriveTrain *driveTrain, RodGrabber *rodGrabber, FieldController *controller);

  int getState();

  virtual bool8 isFinished();
  virtual void update();
};

#endif
