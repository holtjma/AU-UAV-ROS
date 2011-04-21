/*
standardDefs.h
This file is meant to contain things that are used across multiple executables that don't change
*/

#ifndef STANDARD_DEFS_H
#define STANDARD_DEFS_H

#include <math.h>

//TODO:five somethings, feet meters yards?
#define COLLISION_THRESHOLD 5

namespace AU_UAV_ROS
{
	struct waypoint
	{
		double latitude;
		double longitude;
		double altitude;
	};
}

#endif
