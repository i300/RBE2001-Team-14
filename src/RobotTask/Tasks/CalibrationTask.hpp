#include "../../Utilities.h"
#include "../../Subsystems/DriveTrain/DriveTrain.hpp"
#include "../../Subsystems/RodGrabber/RodGrabber.hpp"
#include "../../FieldController/FieldController.hpp"
#include "../RobotTask.hpp"

#ifndef CALIBRATION_H
#define CALIBRATION_H

enum CalibrationState {
  CS_SWING_LEFT,
  CS_SWING_RIGHT,
  CS_SWING_FINAL,
  CS_FINISHED
};

class CalibrationTask : public RobotTask {
private:
  typedef RobotTask super;

protected:
  CalibrationState state;
  unsigned long timeLastStateSwitch = 0;

  const int SWEEP_TIME = 750; // ms

  DriveTrain *_driveTrain;
  RodGrabber *_rodGrabber;
  FieldController *_fieldController;

  void finished();

public:
  CalibrationTask(DriveTrain *driveTrain, RodGrabber *rodGrabber, FieldController *controller);

  int getState();

  bool8 isFinished();
  void update();
};

#endif
