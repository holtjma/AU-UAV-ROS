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

/*
standard constructor
*/
AU_UAV_ROS::PlaneCoordinator::PlaneCoordinator()
{
	//initialize command message index
	this->commandIndex = 0;
}

/*
goToPoint(...)
This function will take a given waypoint and set that as the current and only destination of the UAV

@return: returns true if the function executes without error
*/
bool AU_UAV_ROS::PlaneCoordinator::goToPoint(struct AU_UAV_ROS::waypoint receivedPoint, bool isAvoidanceManeuver, bool isNewQueue)
{
	//clear the queues
	if(isNewQueue)
	{
		std::queue<struct AU_UAV_ROS::waypoint> emptyQueue;
		
		//always clear the avoidance path if we say to do a new queue
		this->avoidancePath = emptyQueue;
		
		if(!isAvoidanceManeuver)
		{
			//only clear the normal path if we specify the normal path
			this->normalPath = emptyQueue;
		}
	}
	
	//add the single waypoint to the specified path
	if(isAvoidanceManeuver)
	{
		this->avoidancePath.push(receivedPoint);
	}
	else
	{
		this->normalPath.push(receivedPoint);
	}
	
	//no ways to error right now so just return true
	return true;
}

/*
loadPathFromFile(...)
This function will load a single path for a plane from file into the normal path queue for the UAV to fly on

@return: returns true if the function executes without error, typical failure is bad filename
*/
bool AU_UAV_ROS::PlaneCoordinator::loadPathfromFile(std::string filename)
{
	return false;
}

/*
handleNewUpdate(...)
Takes a received telemetry update and determine if a new command is necessary
*/
bool AU_UAV_ROS::PlaneCoordinator::handleNewUpdate(AU_UAV_ROS::TelemetryUpdate update, AU_UAV_ROS::Command *newCommand)
{
	bool isCommand = false;
	bool isAvoid;
	
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
	
	//first, check the avoidance queue, since survival is priority #1
	if(!avoidancePath.empty())
	{
		destination = avoidancePath.front();
		isCommand = true;
		isAvoid = true;
	}
	
	//avoidance queue is empty so check normal pathing
	else if(!normalPath.empty())
	{
		//get the first waypoint in the normal path
		destination = normalPath.front();
		isCommand = true;
		isAvoid = false;
	}
	
	if(isCommand)
	{
		//first, check to make sure the plane has the correct current waypoint
		if(distanceBetween(destination, planeDest) > COLLISION_THRESHOLD)
		{
			//the current waypoint is incorrect somehow, send corrective command
			newCommand->commandHeader.seq = this->commandIndex++;
			newCommand->commandHeader.stamp = ros::Time::now();
			newCommand->planeID = this->latestUpdate.planeID;
			newCommand->latitude = destination.latitude;
			newCommand->longitude = destination.longitude;
			newCommand->altitude = destination.altitude;
			return true;
		}
		
		//if the plane has arrived at the current waypoint
		if(distanceBetween(destination, current) < COLLISION_THRESHOLD)
		{
			//remove the waypoint from the queue
			if(isAvoid)
			{
				avoidancePath.pop();
				
				//check for more avoidance waypoint
				if(avoidancePath.empty())
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
					newCommand->latitude = avoidancePath.front().latitude;
					newCommand->longitude = avoidancePath.front().longitude;
					newCommand->altitude = avoidancePath.front().altitude;
					return true;
				}
			}
			else
			{
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
		}
		
		//if we made it here just return
		return false;
	}
	
	//if we get here, then both queues are empty so just return
	ROS_INFO("Plane #%d has no commands right now.", this->latestUpdate.planeID);
	return false;
}
