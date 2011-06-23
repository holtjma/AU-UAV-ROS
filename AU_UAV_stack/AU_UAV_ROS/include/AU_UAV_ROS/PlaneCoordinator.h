/*
PlaneCoordinator
Class responsible for storing:
A) The latest update from the plane
B) The waypoints to go to (aka path)
C) Any collision avoidance waypoints
*/

#ifndef PLANE_COORDINATOR_H
#define PLANE_COORDINATOR_H

//normal headers
#include <stdio.h>
#include <list>
#include <string>

//ROS headers
#include "AU_UAV_ROS/standardDefs.h"
#include "AU_UAV_ROS/TelemetryUpdate.h"
#include "AU_UAV_ROS/Command.h"

namespace AU_UAV_ROS
{
	class PlaneCoordinator
	{
	private:
		//the most recent update received from a plane
		AU_UAV_ROS::TelemetryUpdate latestUpdate;
		
		//two queues related to where the plane should go next, note that avoidance takes priority
		std::list<struct AU_UAV_ROS::waypoint> normalPath;
		std::list<struct AU_UAV_ROS::waypoint> avoidancePath;
		
		//index of the next command to send, starts at 0
		int commandIndex;
		
	public:
		//this is set if the coordinator has allocated this UAV to somewhere
		bool isActive;
		
		//constructors
		PlaneCoordinator();
		
		//command related functions
		bool goToPoint(struct AU_UAV_ROS::waypoint receivedPoint, bool isAvoidanceManeuver, bool isNewQueue);
		struct AU_UAV_ROS::waypoint getWaypointOfQueue(bool isAvoidanceQueue, int position);
		AU_UAV_ROS::Command getPriorityCommand();
		
		//update related functions
		bool handleNewUpdate(AU_UAV_ROS::TelemetryUpdate update, AU_UAV_ROS::Command *newCommand);
	};
}

#endif
