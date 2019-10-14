#include "ExerciseProvider.h"
MAKE_MODULE(ExerciseProvider , behaviorControl)

ExerciseProvider::ExerciseProvider (){}

void ExerciseProvider::update(Exercise &exercise){
  exercise.ballPose = theBallPercept.positionOnField;
  exercise.robotPose = theRobotPose.translation;
  exercise.jointAngles = theJointAngles.angles;

  std::cout << "ballpose = " << exercise.ballPose.transpose() << std::endl;
  std::cout << "robotpose = " << exercise.robotPose.transpose() << std::endl;
  
}