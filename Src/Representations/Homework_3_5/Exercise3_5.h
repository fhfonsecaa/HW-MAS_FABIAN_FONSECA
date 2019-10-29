/****************************************************
* Exercise3_5.h                                       *
* @author   Fabian Fonseca fhfonsecaa@gmail.com     *
* @date     Oct 2019                                *
****************************************************/

#pragma once

#include "Tools/Streams/Enum.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Streams/AutoStreamable.h"

#include "Tools/Math/Angle.h"
#include "Tools/RobotParts/Joints.h"
#include "Tools/Streams/EnumIndexedArray.h"


STREAMABLE(Exercise3_5,
{
	public:
		float _initMacro,
    // EXERCISE 1
    (Vector2f)(0.f, 0.f) robotPose,
    (Vector2f)(0.f, 0.f) ballPose,
    (ENUM_INDEXED_ARRAY(Angle, Joints::Joint)) jointAngles,
    // EXERCISE 2
    (bool)(true) BallPerceptStatus,
    }
});