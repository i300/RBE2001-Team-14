#include "../../Utilities.h"
#include "../../Subsystems/DriveTrain/DriveTrain.hpp"
#include "../../Subsystems/RodGrabber/RodGrabber.hpp"
#include "../../FieldController/FieldController.hpp"
#include "../RobotTask.hpp"

#ifndef STORE_ROD_TASK_H
#define STORE_ROD_TASK_H

enum StoreRodState {
  SRR_DRIVE_TO_LINE,
  SRR_TURN_ONTO_LINE,
  SRR_REALIGN_WITH_LINE,
  SRR_DRIVE_TO_STORAGE,
  SRR_INSERT_ROD,
  SRR_TURN_AROUND,
  SRR_FINISHED
};

class StoreRodTask : public RobotTask {
private:
  typedef RobotTask super;

protected:
  StoreRodState state;
  unsigned long timeLastStateSwitch = 0;

  int8 _rodLocation;

  DriveTrain *_driveTrain;
  RodGrabber *_rodGrabber;
  FieldController *_fieldController;

  void finished();

public:
  StoreRodTask(int8 rodLocation, DriveTrain *driveTrain, RodGrabber *rodGrabber, FieldController *controller);

  int getState();

  bool8 isFinished();
  void update();
};

#endif
