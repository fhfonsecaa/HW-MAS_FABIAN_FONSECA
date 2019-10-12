#include "Exercise1Provider.h"
MAKE_MODULE(Exercise1Provider , behaviorControl)

Exercise1Provider::Exercise1Provider (){}

void Exercise1Provider::update(Exercise1 &exercise1){
  exercise1.ballPose = theBallPercept.positionOnField;
  exercise1.robotPose = theRobotPose.translation;

  std::cout << "ballpose = " << exercise1.ballPose.transpose() << std::endl;
  std::cout << "robotpose = " << exercise1.robotPose.transpose() << std::endl;
}