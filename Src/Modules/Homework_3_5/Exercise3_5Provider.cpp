#include "Exercise3_5Provider.h"
MAKE_MODULE(Exercise3_5Provider, homework_3_5)

Exercise3_5Provider::Exercise3_5Provider (){}

void Exercise3_5Provider::update(Exercise3_5 &exercise3_5){
  exercise3_5.robotPose = theRobotPose.translation;
  exercise3_5.ballPose = theBallModel.lastPerception;

  exercise3_5.ballPoseAproach.x() = theBallModel.lastPerception.x() - theBallState.radius;
  exercise3_5.ballPoseAproach.y() = theBallModel.lastPerception.y() - theBallState.radius;

//   std::cout << "robotpose = " << exercise3_5.robotPose.transpose() << std::endl;
}