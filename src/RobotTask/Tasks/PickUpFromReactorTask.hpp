#include "../../Utilities.h"
#include "../../Subsystems/DriveTrain/DriveTrain.hpp"
#include "../../Subsystems/RodGrabber/RodGrabber.hpp"
#include "../../FieldController/FieldController.hpp"
#include "../RobotTask.hpp"

#ifndef PickUpFromReactor_H
#define PickUpFromReactor_H

enum PickUpFromReactorState {
  PFR_DRIVE_FORWARD,
  PFR_PICK_UP,
  PFR_TURN_AROUND,
  PFR_FINISHED
};

class PickUpFromReactorTask : public RobotTask {
private:
  typedef RobotTask super;

protected:
  PickUpFromReactorState state;
  unsigned long timeLastStateSwitch = 0;

  DriveTrain *_driveTrain;
  RodGrabber *_rodGrabber;
  FieldController *_fieldController;

  void finished();

public:
  PickUpFromReactorTask(DriveTrain *driveTrain, RodGrabber *rodGrabber, FieldController *controller);

  int getState();

  bool8 isFinished();
  void update();
};

#endif
