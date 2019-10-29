#include "ExerciseProvider.h"
MAKE_MODULE(ExerciseProvider, behaviorControl)

ExerciseProvider::ExerciseProvider (){}

void ExerciseProvider::update(Exercise &exercise){
  exercise.ballPose = theBallPercept.positionOnField;
  exercise.robotPose = theRobotPose.translation;
  exercise.jointAngles = theJointAngles.angles;

  exercise.BallPerceptStatus = theBallPercept.status;


  // UNCOMENT EXERCISE 1
  // std::cout << "ballpose = " << exercise.ballPose.transpose() << std::endl;
  // std::cout << "robotpose = " << exercise.robotPose.transpose() << std::endl;
  // for(int index = 0; index < (int)exercise.jointAngles.size(); index++){
  //   std::cout << "jointangle[" << index << "] = "<< exercise.jointAngles.at(index) << std::endl;
  // }

  // UNCOMENT EXERCISE 2
  // std::cout << "ballpose = " << exercise.ballPose.transpose() << std::endl;
  // std::cout << "ballperceptionstatus = " << exercise.BallPerceptStatus << std::endl;
}