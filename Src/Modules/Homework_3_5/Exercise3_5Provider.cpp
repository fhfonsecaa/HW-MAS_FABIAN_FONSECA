#include "Exercise3_5Provider.h"

int robotZone_1 = 0, robotZone_2 = 0;
double margin = 230;
double marginCloser = 130;
double radius = 600;
double footOffset = 90;

MAKE_MODULE(Exercise3_5Provider, homework_3_5)

Exercise3_5Provider::Exercise3_5Provider (){
  corners.push_back(std::tuple<float,float>(-4500,-3000)); //Zone 1
  corners.push_back(std::tuple<float,float>(4500,-3000));  //Zone 2
  corners.push_back(std::tuple<float,float>(4500,3000));   //Zone 3
  corners.push_back(std::tuple<float,float>(-4500,3000));  //Zone 4

  selectCorner();

  homePoses.push_back(std::tuple<float,float>(-4500,1500));  //Robot 1
  homePoses.push_back(std::tuple<float,float>(-4500,-1500)); //Robot 2

  targetPoses.push_back(std::tuple<float,float>(2250,-750)); //Robot 1
  targetPoses.push_back(std::tuple<float,float>(2250,750));  //Robot 2

  ballZone = 0;
}

void Exercise3_5Provider::selectCorner(){
  srand((int)time(0));
  randCornerIndex = (rand() % corners.size()) + 0;
}

bool Exercise3_5Provider::inZone(int zoneIndex, Vector2f pos){
  return (abs(pos.x()) < abs(std::get<0>(corners.at(zoneIndex))) &&
          abs(pos.x()) >= 0 &&
          abs(pos.y()) < abs(std::get<1>(corners.at(zoneIndex))) &&
          abs(pos.y()) >= 0 &&
          pos.x()*std::get<0>(corners.at(zoneIndex)) > 0 &&
          pos.y()*std::get<1>(corners.at(zoneIndex)) > 0);
}

bool Exercise3_5Provider::inZone(int zoneIndex, Pose2f pos){
  return (abs(pos.translation.x()) < abs(std::get<0>(corners.at(zoneIndex))) &&
          abs(pos.translation.x()) >= 0 &&
          abs(pos.translation.y()) < abs(std::get<1>(corners.at(zoneIndex))) &&
          abs(pos.translation.y()) >= 0 &&
          pos.translation.x()*std::get<0>(corners.at(zoneIndex)) > 0 &&
          pos.translation.y()*std::get<1>(corners.at(zoneIndex)) > 0);
}

void Exercise3_5Provider::update(Exercise3_5 &exercise3_5){
  exercise3_5.robotPose = theRobotPose.translation;
  exercise3_5.ballPose = theBallModel.lastPerception;

  double orientation = atan2 (exercise3_5.ballPose.y(),exercise3_5.ballPose.x());


  /* ====================================== EXERCISE 3 ====================================== */
  /* ======================================= Aproach ======================================== */

  exercise3_5.ballPoseAproach.x() = theBallModel.lastPerception.x() - margin*cos(orientation);
  exercise3_5.ballPoseAproach.y() = theBallModel.lastPerception.y() - margin*sin(orientation);
  exercise3_5.distanceToBall = hypot (theBallModel.lastPerception.x(), theBallModel.lastPerception.y());

  // std::cout << "robot position = " << exercise3_5.robotPose << std::endl;
  // std::cout << "ball position = " << exercise3_5.ballPose.x() <<  "  " << exercise3_5.ballPose.y() << std::endl;
  // std::cout << "ball aproach = " << exercise3_5.ballPoseAproach.x() <<  "  " << exercise3_5.ballPoseAproach.y() << std::endl;
  // std::cout << "Radius  = " << theBallModel.estimate.radius << " X " << theBallModel.estimate.radius*cos(orientation) << " Y " << theBallModel.estimate.radius*sin(orientation) << std::endl;
  // std::cout << "ball orientation = " << orientation * 180 / PI << " grad" << std::endl;
  // std::cout << "ball distance = " << exercise3_5.distanceToBall << std::endl;
  
  /* ====================================== EXERCISE 3 ====================================== */
  /* ================================== Circle trajectory =================================== */

  exercise3_5.ballPoseCircle.x() = theBallModel.lastPerception.x() + cos(orientation)*radius;
  exercise3_5.ballPoseCircle.y() = theBallModel.lastPerception.y() + sin(orientation)*radius;

  /* ====================================== EXERCISE 4 ====================================== */
  /* =================================== Kick to corner ===================================== */

  // std::cout << "corner[" << randCornerIndex << "] = " << 
  //              std::get<0>(corners.at(randCornerIndex)) << 
  //              std::get<1>(corners.at(randCornerIndex)) << std::endl;
  Pose2f globBallPose = theLibCodeRelease.rel2Glob(theBallModel.lastPerception.x(),theBallModel.lastPerception.y());
  double ballToCornerOrient = atan2 (std::get<1>(corners.at(randCornerIndex))-globBallPose.translation.y(),
                                     std::get<0>(corners.at(randCornerIndex))-globBallPose.translation.x());
  
  exercise3_5.ballPoseToKick.x() = globBallPose.translation.x() - margin*cos(ballToCornerOrient) + footOffset*sin(ballToCornerOrient);
  exercise3_5.ballPoseToKick.y() = globBallPose.translation.y() - margin*sin(ballToCornerOrient) + footOffset*cos(ballToCornerOrient);
  exercise3_5.angleToKick = ballToCornerOrient;
  // std::cout << "robot position = " << exercise3_5.robotPose.x() <<  "  " << exercise3_5.robotPose.y() << std::endl;
  // std::cout << "target position = " << exercise3_5.ballPoseToKick.x() <<  "  " << exercise3_5.ballPoseToKick.y() << std::endl;
  // std::cout << "ball position = " << globBallPose.translation.x() <<  "  " << globBallPose.translation.y() << std::endl;
  // std::cout << "actual orientation = " << theRobotPose.rotation << " grad" << std::endl;
  // std::cout << "foot offset = " << footOffset*sin(ballToCornerOrient) <<  "  " << footOffset*cos(ballToCornerOrient) << std::endl;
  // std::cout << "goal orientation = " << ballToCornerOrient * 180 / PI << " grad" << std::endl;


  /* ====================================== EXERCISE 5 ====================================== */
  /* ==================================== Ping Pong Game ==================================== */

  if(inZone(3,theLibCodeRelease.rel2Glob(theBallModel.lastPerception.x(),theBallModel.lastPerception.y())) || 
     inZone(2,theLibCodeRelease.rel2Glob(theBallModel.lastPerception.x(),theBallModel.lastPerception.y())))
    ballZone = 2;
  else if(inZone(1,theLibCodeRelease.rel2Glob(theBallModel.lastPerception.x(),theBallModel.lastPerception.y())) || 
          inZone(0,theLibCodeRelease.rel2Glob(theBallModel.lastPerception.x(),theBallModel.lastPerception.y())))
    ballZone = 1;

  if(theRobotInfo.number == 1){
    // std::cout << "robot " << theRobotInfo.number << " position = " << exercise3_5.robotPose.x() <<  "  " << exercise3_5.robotPose.y() << std::endl;
    // std::cout << "robot " << theRobotInfo.number << " zone = " << robotZones.at(0);
    exercise3_5.homePose = Vector2f(std::get<0>(homePoses.at(0)),std::get<1>(homePoses.at(0)));

    if(inZone(3,exercise3_5.robotPose))
      robotZone_1 = 4;
    else if(inZone(2,exercise3_5.robotPose))
      robotZone_1 = 3;
    
    if(ballZone == 1)
      exercise3_5.strRole = "taker";
    else if(ballZone == 2 &&
            robotZone_2 == 1){
      Pose2f globBallPose = theLibCodeRelease.rel2Glob(theBallModel.lastPerception.x(),theBallModel.lastPerception.y());
      double ballToCornerOrient = atan2 (std::get<1>(targetPoses.at(0))-globBallPose.translation.y(),
                                         std::get<0>(targetPoses.at(0))-globBallPose.translation.x());

      exercise3_5.ballPoseToKick.x() = globBallPose.translation.x() - margin*cos(ballToCornerOrient) + footOffset*sin(ballToCornerOrient);
      exercise3_5.ballPoseToKick.y() = globBallPose.translation.y() - margin*sin(ballToCornerOrient) + footOffset*cos(ballToCornerOrient);
      exercise3_5.ballPoseToKickCloser.x() = globBallPose.translation.x() - marginCloser*cos(ballToCornerOrient) + footOffset*sin(ballToCornerOrient);
      exercise3_5.ballPoseToKickCloser.y() = globBallPose.translation.y() - marginCloser*sin(ballToCornerOrient) + footOffset*cos(ballToCornerOrient);
      exercise3_5.angleToKick = ballToCornerOrient;

      exercise3_5.strRole = "kicker";
    }
  }else if(theRobotInfo.number == 2){
    // std::cout << "robot " << theRobotInfo.number << " position = " << exercise3_5.robotPose.x() <<  "  " << exercise3_5.robotPose.y() << std::endl;
    // std::cout << "robot " << theRobotInfo.number << " zone = " << robotZones.at(1) << std::endl;
    exercise3_5.homePose = Vector2f(std::get<0>(homePoses.at(1)),std::get<1>(homePoses.at(1)));

    if(inZone(1,exercise3_5.robotPose))
      robotZone_2 = 2;
    else if(inZone(0,exercise3_5.robotPose))
      robotZone_2 = 1;
      
    if(ballZone == 2)
      exercise3_5.strRole = "taker";
    else if(ballZone == 1 &&
            robotZone_1 == 4){
      Pose2f globBallPose = theLibCodeRelease.rel2Glob(theBallModel.lastPerception.x(),theBallModel.lastPerception.y());
      double ballToCornerOrient = atan2 (std::get<1>(targetPoses.at(1))-globBallPose.translation.y(),
                                         std::get<0>(targetPoses.at(1))-globBallPose.translation.x());

      exercise3_5.ballPoseToKick.x() = globBallPose.translation.x() - margin*cos(ballToCornerOrient) + footOffset*sin(ballToCornerOrient);
      exercise3_5.ballPoseToKick.y() = globBallPose.translation.y() - margin*sin(ballToCornerOrient) + footOffset*cos(ballToCornerOrient);
      exercise3_5.ballPoseToKickCloser.x() = globBallPose.translation.x() - marginCloser*cos(ballToCornerOrient) + footOffset*sin(ballToCornerOrient);
      exercise3_5.ballPoseToKickCloser.y() = globBallPose.translation.y() - marginCloser*sin(ballToCornerOrient) + footOffset*cos(ballToCornerOrient);
      exercise3_5.angleToKick = ballToCornerOrient;

      exercise3_5.strRole = "kicker";

    }
  }
}  