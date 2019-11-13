 /* ====================================== EXERCISE 5 ====================================== */
 /* ==================================== Ping Pong Game ==================================== */
 /* ======================================== Kicker ======================================== */
option(KickerBehavior)
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
        goto goToBall;
    }
    action
    {
      lookAtBall();
      WalkAtRelativeSpeed(Pose2f(0.6f, 0.f, 0.f));
    }
  }

  state(goToBall)
  {
    std::cout << "GoToBall" <<std::endl;
    transition
    {
      if (theLibCodeRelease.timeSinceBallWasSeen > 3000)
        goto lookAround;
      if (abs(theExercise3_5.ballPoseToKick.x() - theRobotPose.translation.x()) < 30 &&
          abs(theExercise3_5.ballPoseToKick.y() - theRobotPose.translation.y()) < 30 &&
          abs(theExercise3_5.angleToKick - theRobotPose.rotation) < 0.05)
        goto kickToZone;
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

  state(kickToZone)
  {
    std::cout << "kickToZone" <<std::endl;
    std::cout << state_time <<std::endl;
    transition
    {
      if (theLibCodeRelease.timeSinceBallWasSeen > 3000)
        goto lookAround;
      if (state_time > 2000)
        goto goToBallCloser;
    }
    action
    {
      lookAtBall();
      Kicks(std::string("lobKick"));
    }
  }

  state(goToBallCloser)
  {
    std::cout << "goToBallCloser" <<std::endl;
    transition
    {
      if (theLibCodeRelease.timeSinceBallWasSeen > 3000)
        goto lookAround;
      if (abs(theExercise3_5.ballPoseToKickCloser.x() - theRobotPose.translation.x()) < 10 &&
          abs(theExercise3_5.ballPoseToKickCloser.y() - theRobotPose.translation.y()) < 10 &&
          abs(theExercise3_5.angleToKick - theRobotPose.rotation) < 0.05)
        goto kickToZone;
    }
    action
    {
      lookAtBall();
      WalkToTargetPathPlanner(Pose2f(1.f, 1.f, 1.f),
                              Pose2f(Angle(theExercise3_5.angleToKick),
                                     theExercise3_5.ballPoseToKickCloser.x(),
                                     theExercise3_5.ballPoseToKickCloser.y()));
    }
  }
}