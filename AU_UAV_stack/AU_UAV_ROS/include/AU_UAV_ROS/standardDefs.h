/*
standardDefs.h
This file is meant to contain things that are used across multiple executables that don't change
*/

#ifndef STANDARD_DEFS_H
#define STANDARD_DEFS_H

#include <math.h>

//TODO: .0001 somethings with respect to longitude and latitude, needs to be changed
#define COLLISION_THRESHOLD .0001

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
