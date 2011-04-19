/*
PlaneCoordinator
Class responsible for storing:
A) The latest update from the plane
B) The waypoints to go to (aka path)
C) Any collision avoidance waypoints
*/

#include <stdio.h>

#include "AU_UAV_ROS/PlaneCoordinator.h"

bool AU_UAV_ROS::PlaneCoordinator::handleNewUpdate(AU_UAV_ROS::TelemetryUpdate update)
{
	//store the last update
	this->latestUpdate = update;
	
	//TODO:check to see if we need to send a new command before returning
	
	
	return true;
}
