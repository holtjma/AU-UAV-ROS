/*
PlaneCoordinator
Class responsible for storing:
A) The latest update from the plane
B) The waypoints to go to (aka path)
C) Any collision avoidance waypoints
*/

#include <stdio.h>
#include <queue>

#include "AU_UAV_ROS/standardDefs.h"
#include "AU_UAV_ROS/TelemetryUpdate.h"

namespace AU_UAV_ROS
{
	class PlaneCoordinator
	{
	private:
		AU_UAV_ROS::TelemetryUpdate latestUpdate;
		std::queue<struct waypoint> normalPath;
		std::queue<struct waypoint> avoidancePath;

	public:
		bool handleNewUpdate(AU_UAV_ROS::TelemetryUpdate update);
	};
}
