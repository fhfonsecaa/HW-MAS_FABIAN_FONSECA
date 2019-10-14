/****************************************************
* ExerciseProvider.h                               *
* @author   Fabian Fonseca fhfonsecaa@gmail.com     *
* @date     Oct 2019                                *
****************************************************/

#include "Representations/BehaviorControl/Exercise.h"
#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Tools/Module/Module.h"
#include <iostream>

MODULE (ExerciseProvider,
{,
  REQUIRES(BallPercept),
  REQUIRES(RobotPose),
  REQUIRES(JointAngles),
  PROVIDES(Exercise),
});

class ExerciseProvider : public ExerciseProviderBase {
public :
  ExerciseProvider();
  void update(Exercise &exercise);
};