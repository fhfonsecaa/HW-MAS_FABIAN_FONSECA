/****************************************************
* Exercise1Provider.h                               *
* @author   Fabian Fonseca fhfonsecaa@gmail.com     *
* @date     Oct 2019                                *
****************************************************/
#ifndef __EXERCISE1PROVIDER__
#define __EXERCISE1PROVIDER__


#include "Representations/spqr_representations/Exercise1.h"
#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/Module/Module.h"
#include <iostream>

MODULE (Exercise1Provider,
{,
  REQUIRES(BallPercept),
  REQUIRES(RobotPose),
  PROVIDES(Exercise1),
});

class Exercise1Provider : public Exercise1ProviderBase { //: public Exercise1Provider {
public :
  Exercise1Provider();
  void update(Exercise1 &exercise1);
};
#endif