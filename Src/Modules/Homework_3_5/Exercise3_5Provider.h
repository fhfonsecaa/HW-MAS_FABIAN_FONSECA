/****************************************************
* ExerciseProvider3_5.h                               *
* @author   Fabian Fonseca fhfonsecaa@gmail.com     *
* @date     Oct 2019                                *
****************************************************/

#include "Representations/Homework_3_5/Exercise3_5.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Tools/Module/Module.h"
#include <iostream>

#define PI 3.14159265
#define dA 0.00001

MODULE (Exercise3_5Provider,
{,
  REQUIRES(BallModel),
  REQUIRES(RobotPose),
  PROVIDES(Exercise3_5),
});

class Exercise3_5Provider : public Exercise3_5ProviderBase {
  public:
    Exercise3_5Provider();
    void update(Exercise3_5 &exercise3_5);
  private:
    double angle;
};