#include "Exercise3_5Provider.h"
MAKE_MODULE(Exercise3_5Provider, homework_3_5)

Exercise3_5Provider::Exercise3_5Provider (){}

void Exercise3_5Provider::update(Exercise3_5 &exercise3_5){
  exercise3_5.robotPose = theRobotPose.translation;
  exercise3_5.ballPose = theBallModel.lastPerception;

  double orientation = atan2 (exercise3_5.ballPose.y(),exercise3_5.ballPose.x());
  double margin = 220;
  double radius = 600;

  exercise3_5.ballPoseAproach.x() = theBallModel.lastPerception.x() - margin*cos(orientation);
  exercise3_5.ballPoseAproach.y() = theBallModel.lastPerception.y() - margin*sin(orientation);
  exercise3_5.distanceToBall = hypot (theBallModel.lastPerception.x(), theBallModel.lastPerception.y());

  // std::cout << "robot position = " << exercise3_5.robotPose << std::endl;
  // std::cout << "ball position = " << exercise3_5.ballPose.x() <<  "  " << exercise3_5.ballPose.y() << std::endl;
  // std::cout << "ball aproach = " << exercise3_5.ballPoseAproach.x() <<  "  " << exercise3_5.ballPoseAproach.y() << std::endl;
  std::cout << "ball distance = " << exercise3_5.distanceToBall << std::endl;
  
  // std::cout << "ball orientation = " << orientation * 180 / PI << " grad" << std::endl;

  // std::cout << "Radius  = " << theBallModel.estimate.radius << " X " << theBallModel.estimate.radius*cos(orientation) << " Y " << theBallModel.estimate.radius*sin(orientation) << std::endl;

  exercise3_5.ballPoseCircle.x() = theBallModel.lastPerception.x() + cos(orientation)*radius;
  exercise3_5.ballPoseCircle.y() = theBallModel.lastPerception.y() + sin(orientation)*radius;
}