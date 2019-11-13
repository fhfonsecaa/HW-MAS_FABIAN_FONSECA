 /* ====================================== EXERCISE 5 ====================================== */
 /* ==================================== Ping Pong Game ==================================== */
 /* ======================================== Taker ========================================= */
option(TakerBehavior)
{
  initial_state(start)
  {
    transition
    {
      if (theLibCodeRelease.timeSinceBallWasSeen > 3000)
        goto lookAround;
      else
        goto goToHome;
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
        goto goToHome;
    }
    action
    {
      lookAtBall();
      WalkAtRelativeSpeed(Pose2f(0.6f, 0.f, 0.f));
    }
  }

  state(goToHome)
  {
    transition
    {
      if (theLibCodeRelease.timeSinceBallWasSeen > 3000)
        goto lookAround;
    }
    action
    {
      WalkToTargetPathPlanner(Pose2f(1.f, 1.f, 1.f),
                              Pose2f(Angle(0),
                                     theExercise3_5.homePose.x(),
                                     theExercise3_5.homePose.y()));
    }
  }
}