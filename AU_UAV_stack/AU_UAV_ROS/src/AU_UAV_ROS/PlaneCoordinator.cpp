/*
PlaneCoordinator
Class responsible for storing:
A) The latest update from the plane
B) The waypoints to go to (aka path)
C) Any collision avoidance waypoints
*/

#include <stdio.h>
#include <queue>

#include "ros/ros.h"

#include "AU_UAV_ROS/standardDefs.h"
#include "AU_UAV_ROS/PlaneCoordinator.h"
#include "AU_UAV_ROS/Command.h"

double distanceBetween(struct AU_UAV_ROS::waypoint first, struct AU_UAV_ROS::waypoint second)
{
	return sqrt(pow(first.latitude - second.latitude, 2) + pow(first.longitude - second.longitude, 2) + pow(first.altitude - second.altitude, 2));
}

AU_UAV_ROS::PlaneCoordinator::PlaneCoordinator()
{
	//initialize command message index
	this->commandIndex = 0;
}

/*
handleNewUpdate(...)
Takes a received telemetry update and determine if a new command is necessary
*/
bool AU_UAV_ROS::PlaneCoordinator::handleNewUpdate(AU_UAV_ROS::TelemetryUpdate update, AU_UAV_ROS::Command *newCommand)
{
	//store the last update
	this->latestUpdate = update;
	
	struct waypoint destination;
	struct waypoint current;
	struct waypoint planeDest;
	
	current.latitude = update.currentLatitude;
	current.longitude = update.currentLongitude;
	current.altitude = update.currentAltitude;

	planeDest.latitude = update.destLatitude;
	planeDest.longitude = update.destLongitude;
	planeDest.altitude = update.destAltitude;
	
	
	//TODO:check to see if we need to send a new command before returning
	//first, check the avoidance queue, since survival is priority #1
	if(!avoidancePath.empty())
	{
		//TODO: something!
		return false;
	}
	 
	//avoidance queue is empty so check normal pathing
	if(!normalPath.empty())
	{
		//get the first waypoint in the normal path
		destination = normalPath.front();
		
		//first, check to make sure the plane has the correct current waypoint
		if(distanceBetween(destination, planeDest) > COLLISION_THRESHOLD)
		{
			//the current waypoint is incorrect somehow, send corrective command
			newCommand->commandHeader.seq = this->commandIndex++;
			newCommand->commandHeader.stamp = ros::Time::now();
			newCommand->planeID = this->latestUpdate.planeID;
			newCommand->latitude = normalPath.front().latitude;
			newCommand->longitude = normalPath.front().longitude;
			newCommand->altitude = normalPath.front().altitude;
			return true;
		}
		
		//if the plane has arrived at the current waypoint
		if(distanceBetween(destination, current) < COLLISION_THRESHOLD)
		{
			//remove the waypoint from the queue
			normalPath.pop();
			
			//check for more waypoints in the path
			if(normalPath.empty())
			{
				//no more pathing commands
				return false;
			}
			else
			{
				//fill data for next normal waypoint
				newCommand->commandHeader.seq = this->commandIndex++;
				newCommand->commandHeader.stamp = ros::Time::now();
				newCommand->planeID = this->latestUpdate.planeID;
				newCommand->latitude = normalPath.front().latitude;
				newCommand->longitude = normalPath.front().longitude;
				newCommand->altitude = normalPath.front().altitude;
				return true;
			}
		}
		
		//if we made it here just return
		return false;
	}
	
	//if we get here, then both queues are empty so just return
	ROS_INFO("Plane #%d has no commands right now.", this->latestUpdate.planeID);
	return false;
}
