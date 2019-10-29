/****************************************************
* ExerciseProvider3_5.h                               *
* @author   Fabian Fonseca fhfonsecaa@gmail.com     *
* @date     Oct 2019                                *
****************************************************/

#include "Representations/Homework_3_5/Exercise3_5.h"
#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Tools/Module/Module.h"
#include <iostream>

MODULE (Exercise3_5Provider,
{,
  REQUIRES(BallPercept),
  REQUIRES(RobotPose),
  REQUIRES(JointAngles),
  PROVIDES(Exercise3_5),
});

class Exercise3_5Provider : public Exercise3_5ProviderBase {
public :
  Exercise3_5Provider();
  void update(Exercise3_5 &exercise3_5);
};