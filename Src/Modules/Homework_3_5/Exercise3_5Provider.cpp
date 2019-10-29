#include "Exercise3_5Provider.h"
MAKE_MODULE(Exercise3_5Provider, homework_3_5)

Exercise3_5Provider::Exercise3_5Provider (){}

void Exercise3_5Provider::update(Exercise3_5 &exercise3_5){
  exercise3_5.ballPose = theBallPercept.positionOnField;
  exercise3_5.robotPose = theRobotPose.translation;
  exercise3_5.jointAngles = theJointAngles.angles;

  exercise3_5.BallPerceptStatus = theBallPercept.status;

  std::cout << "ballpose = " << exercise3_5.ballPose.transpose() << std::endl;
  std::cout << "robotpose = " << exercise3_5.robotPose.transpose() << std::endl;
}