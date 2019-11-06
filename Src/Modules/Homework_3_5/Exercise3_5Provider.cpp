#include "Exercise3_5Provider.h"
MAKE_MODULE(Exercise3_5Provider, homework_3_5)

Exercise3_5Provider::Exercise3_5Provider (){
  corners.push_back(std::tuple<float,float>(4500,3000));
  corners.push_back(std::tuple<float,float>(4500,-3000));
  corners.push_back(std::tuple<float,float>(-4500,3000));
  corners.push_back(std::tuple<float,float>(-4500,-3000));

  selectCorner();
}

void Exercise3_5Provider::selectCorner(){
  srand((int)time(0));
  randCornerIndex = (rand() % corners.size()+1) + 0;
}

void Exercise3_5Provider::update(Exercise3_5 &exercise3_5){
  exercise3_5.robotPose = theRobotPose.translation;
  exercise3_5.ballPose = theBallModel.lastPerception;

  double orientation = atan2 (exercise3_5.ballPose.y(),exercise3_5.ballPose.x());
  double margin = 200;
  double radius = 600;

  /*Only aproach*/

  exercise3_5.ballPoseAproach.x() = theBallModel.lastPerception.x() - margin*cos(orientation);
  exercise3_5.ballPoseAproach.y() = theBallModel.lastPerception.y() - margin*sin(orientation);
  exercise3_5.distanceToBall = hypot (theBallModel.lastPerception.x(), theBallModel.lastPerception.y());

  // std::cout << "robot position = " << exercise3_5.robotPose << std::endl;
  // std::cout << "ball position = " << exercise3_5.ballPose.x() <<  "  " << exercise3_5.ballPose.y() << std::endl;
  // std::cout << "ball aproach = " << exercise3_5.ballPoseAproach.x() <<  "  " << exercise3_5.ballPoseAproach.y() << std::endl;
  // std::cout << "Radius  = " << theBallModel.estimate.radius << " X " << theBallModel.estimate.radius*cos(orientation) << " Y " << theBallModel.estimate.radius*sin(orientation) << std::endl;
  // std::cout << "ball orientation = " << orientation * 180 / PI << " grad" << std::endl;
  std::cout << "ball distance = " << exercise3_5.distanceToBall << std::endl;
  
  /*Circle trajectory*/

  exercise3_5.ballPoseCircle.x() = theBallModel.lastPerception.x() + cos(orientation)*radius;
  exercise3_5.ballPoseCircle.y() = theBallModel.lastPerception.y() + sin(orientation)*radius;

  /*Kick to corner*/

  std::cout << "corner[" << randCornerIndex << "] = " << 
                std::get<0>(corners.at(randCornerIndex))  << 
                std::get<1>(corners.at(randCornerIndex)) << std::endl;


  Pose2f globBallPose = theLibCodeRelease.rel2Glob(theBallModel.estimate.position.x(),theBallModel.estimate.position.y());
  double ballToCornerOrient = atan2 (std::get<1>(corners.at(randCornerIndex))-globBallPose.translation.y(),
                                     std::get<0>(corners.at(randCornerIndex))-globBallPose.translation.x());
  
  exercise3_5.ballPoseToKick.x() = globBallPose.translation.x() - margin*cos(ballToCornerOrient);
  exercise3_5.ballPoseToKick.y() = globBallPose.translation.y() - margin*sin(ballToCornerOrient);
  exercise3_5.angleToKick = ballToCornerOrient;
  std::cout << "robot position = " << exercise3_5.robotPose.x() <<  "  " << exercise3_5.robotPose.y() << std::endl;
  std::cout << "target position = " << exercise3_5.ballPoseToKick.x() <<  "  " << exercise3_5.ballPoseToKick.y() << std::endl;
  std::cout << "ball position = " << globBallPose.translation.x() <<  "  " << globBallPose.translation.y() << std::endl;
  std::cout << "goal orientation = " << ballToCornerOrient << " grad" << std::endl;
  std::cout << "actual orientation = " << theRobotPose.rotation << " grad" << std::endl;

}  

 

