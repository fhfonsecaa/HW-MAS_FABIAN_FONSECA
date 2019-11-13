/* ====================================== EXERCISE 3 ====================================== */
/* ======================================= Aproach ======================================== */
/* ================================== Circle trajectory =================================== */
option(WalkToHomework)
{
  initial_state(start)
  {
    transition
    {
      if (theLibCodeRelease.timeSinceBallWasSeen > 3000)
        goto lookAround;
      else
        goto walkToBall;
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
        goto walkToBall;
    }
    action
    {
      lookAtBall();
      WalkAtRelativeSpeed(Pose2f(0.6f, 0.f, 0.f));
    }
  }

  state(walkToBall)
  {
    transition
    {
      if (theLibCodeRelease.timeSinceBallWasSeen > 3000)
        goto lookAround ;
    }
    action
    {
      lookAtBall();
      // WalkToTargetPathPlanner(Pose2f(1.f, 1.f, 1.f),
      //                         Pose2f(theLibCodeRelease.rel2Glob(theExercise3_5.ballPoseAproach.x(),
      //                                                        theExercise3_5.ballPoseAproach.y())));
      
      WalkToTargetPathPlanner(Pose2f(1.f, 1.f, 1.f),
                              Pose2f(theLibCodeRelease.rel2Glob(theExercise3_5.ballPoseCircle.x(),
                                                                theExercise3_5.ballPoseCircle.y())));
    }
  }
}