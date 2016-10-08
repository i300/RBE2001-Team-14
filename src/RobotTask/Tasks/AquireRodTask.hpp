#include "../../Utilities.h"
#include "../../Subsystems/DriveTrain/DriveTrain.hpp"
#include "../../Subsystems/RodGrabber/RodGrabber.hpp"
#include "../../FieldController/FieldController.hpp"
#include "../RobotTask.hpp"
/*
 *AquireRodTask
 *Class to store the task of Aquiring the rod
 */

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

protected:
  void finished();

public:
  AquireRodTask(int8 currentLocation, int8 rodLocation, DriveTrain *driveTrain, RodGrabber *rodGrabber, FieldController *controller);

  virtual bool8 isFinished();
  virtual void update();
};

#endif
