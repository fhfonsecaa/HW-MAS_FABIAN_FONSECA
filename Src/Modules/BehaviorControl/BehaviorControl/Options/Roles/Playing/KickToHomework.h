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
      if (theExercise3_5.distanceToBall < 220 &&
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
    }
  }

  // state(alignToFoot)
  // {
  //   Pose2f alignPose = Pose2f(theLibCodeRelease.rel2Glob(-10,0));
  //   alignPose.rotation = theExercise3_5.angleToKick;

  //   transition
  //   {
  //     if (theLibCodeRelease.timeSinceBallWasSeen > 3000)
  //       goto lookAround;
  //     if (abs(alignPose.translation.y() - theRobotPose.translation.y()) < 10 &&
  //         abs(alignPose.translation.x() - theRobotPose.translation.x()) < 10 &&
  //         abs(theExercise3_5.angleToKick - theRobotPose.rotation) < 0.05)
  //       goto kickToCorner;
  //   }
  //   action
  //   {
  //     lookAtBall();
  //     WalkToTargetPathPlanner(Pose2f(1.f, 1.f, 1.f), alignPose);
  //   }
  // }

  state(kickToCorner)
  {
    transition
    {
      if (theLibCodeRelease.timeSinceBallWasSeen > 3000)
        goto lookAround;
      if (theExercise3_5.distanceToBall > 220 ||
          abs(theExercise3_5.angleToKick - theRobotPose.rotation) > 0.05)
        goto goToBall;
    }
    action
    {
      lookAtBall();
      Kicks(std::string("lobKick"));
    }
  }
}