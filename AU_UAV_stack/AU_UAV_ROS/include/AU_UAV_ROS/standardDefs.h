/*
standardDefs.h
This file is meant to contain things that are used across multiple executables that don't change
*/

#ifndef STANDARD_DEFS
#define STANDARD_DEFS

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
