/* ====================================== EXERCISE 4 ====================================== */
/* =================================== Kick to corner ===================================== */
option(KickToHomework)
{
  initial_state(start)
  {
    transition
    {
      if (theLibCodeRelease.timeSinceBallWasSeen > 3000)
        goto lookAround;
      else
        goto goToBall;
    }
    action
    {
      lookAtBall();
      Stand();
    }
  }

  state(lookAround)
  {
    transition
    {
      if (theLibCodeRelease.timeSinceBallWasSeen < 700)
        goto kickToCorner;
    }
    action
    {
      lookAtBall();
      WalkAtRelativeSpeed(Pose2f(0.6f, 0.f, 0.f));
    }
  }

  state(goToBall)
  {
    transition
    {
      if (theLibCodeRelease.timeSinceBallWasSeen > 3000)
        goto lookAround;
      if (abs(theExercise3_5.ballPoseToKick.x() - theRobotPose.translation.x()) < 30 &&
          abs(theExercise3_5.ballPoseToKick.y() - theRobotPose.translation.y()) < 30 &&
          abs(theExercise3_5.angleToKick - theRobotPose.rotation) < 0.05)
        goto kickToCorner;
    }
    action
    {
      lookAtBall();
      WalkToTargetPathPlanner(Pose2f(1.f, 1.f, 1.f),
                              Pose2f(Angle(theExercise3_5.angleToKick),
                                     theExercise3_5.ballPoseToKick.x(),
                                     theExercise3_5.ballPoseToKick.y()));
      std::cout << abs(theExercise3_5.robotPose.y()) << std::endl;

    }
  }

  state(kickToCorner)
  {
    transition
    {
      if (theLibCodeRelease.timeSinceBallWasSeen > 3000)
        goto lookAround;
      if (theExercise3_5.distanceToBall > 300)
        goto goToBall;
    }
    action
    {
      lookAtBall();
      Kicks(std::string("fastForwardKick"));
    }
  }
}