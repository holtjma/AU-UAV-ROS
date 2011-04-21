/*
PlaneCoordinator
Class responsible for storing:
A) The latest update from the plane
B) The waypoints to go to (aka path)
C) Any collision avoidance waypoints
*/

#ifndef PLANE_COORDINATOR_H
#define PLANE_COORDINATOR_H

#include <stdio.h>
#include <queue>

#include "AU_UAV_ROS/standardDefs.h"
#include "AU_UAV_ROS/TelemetryUpdate.h"
#include "AU_UAV_ROS/Command.h"

namespace AU_UAV_ROS
{
	class PlaneCoordinator
	{
	private:
		AU_UAV_ROS::TelemetryUpdate latestUpdate;
		std::queue<struct waypoint> normalPath;
		std::queue<struct waypoint> avoidancePath;
		int commandIndex;
		
	public:
		PlaneCoordinator();
		bool handleNewUpdate(AU_UAV_ROS::TelemetryUpdate update, AU_UAV_ROS::Command *newCommand);
	};
}

#endif
