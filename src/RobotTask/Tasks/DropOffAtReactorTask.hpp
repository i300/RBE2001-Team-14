#include "../../Utilities.h"
#include "../../Subsystems/DriveTrain/DriveTrain.hpp"
#include "../../Subsystems/RodGrabber/RodGrabber.hpp"
#include "../../FieldController/FieldController.hpp"
#include "../RobotTask.hpp"

#ifndef DropOffAtReactor_H
#define DropOffAtReactor_H

enum DropOffAtReactorState {
  DR_DRIVE_TO_MAIN_LINE,
  DR_TURN_TO_REACTOR,
  DR_DRIVE_TO_REACTOR,
  DR_DEPOSIT_ROD,
  DR_TURN_AROUND,
  DR_FINISHED
};

class DropOffAtReactorTask : public RobotTask {
private:
  typedef RobotTask super;

protected:
  DropOffAtReactorState state;
  unsigned long timeLastStateSwitch = 0;

  DriveTrain *_driveTrain;
  RodGrabber *_rodGrabber;
  FieldController *_fieldController;

  int8 _currentReactor;

  void finished();

public:
  DropOffAtReactorTask(int8 currentReactor, DriveTrain *driveTrain, RodGrabber *rodGrabber, FieldController *controller);

  int getState();

  bool8 isFinished();
  void update();
};

#endif
