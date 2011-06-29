/*
PlaneCoordinator
Class responsible for storing:
A) The latest update from the plane
B) The waypoints to go to (aka path)
C) Any collision avoidance waypoints
*/

//normal headers
#include <stdio.h>
#include <list>

//ROS headers
#include "ros/ros.h"
#include "AU_UAV_ROS/standardDefs.h"
#include "AU_UAV_ROS/PlaneCoordinator.h"
#include "AU_UAV_ROS/Command.h"

/*
standard constructor
*/
AU_UAV_ROS::PlaneCoordinator::PlaneCoordinator()
{
	//initialize command message index
	this->commandIndex = 0;
	this->isActive = false;
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
		std::list<struct AU_UAV_ROS::waypoint> emptyQueue;
		
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
		this->avoidancePath.push_back(receivedPoint);
	}
	else
	{
		this->normalPath.push_back(receivedPoint);
	}
	
	//no ways to error right now so just return true
	return true;
}

/*
getWaypointOfQueue(...)
This function will look at the specified queue (avoidance or normal) and return the waypoint
at the position of that queue.  In the case of nothing being there, the waypoint returned will be
(-1000, -1000, -1000) because it is a non-viable waypoint in all three categories.
*/
struct AU_UAV_ROS::waypoint AU_UAV_ROS::PlaneCoordinator::getWaypointOfQueue(bool isAvoidanceQueue, int position)
{
	//setup invalid return as default
	struct AU_UAV_ROS::waypoint ret;
	ret.latitude = -1000;
	ret.longitude = -1000;
	ret.altitude = -1000;
	
	std::list<struct AU_UAV_ROS::waypoint>::iterator ii;
	int count = 0;
	
	//get the right queue
	if(isAvoidanceQueue)
	{
		for(ii = avoidancePath.begin(); ii != avoidancePath.end() && count < position; ii++)
		{
			count++;
		}
		if(ii != avoidancePath.end())
		{
			ret = *ii;
		}
	}
	else
	{
		for(ii = normalPath.begin(); ii != normalPath.end() && count < position; ii++)
		{
			count++;
		}
		if(ii != normalPath.end())
		{
			ret = *ii;
		}
	}
	
	return ret;
}	

/*
getPriorityCommand()
This function will return a filled out command that represents the highest priority command based on the
last known information provided by the UAV.  Will return a point with (-1000, -1000, -1000) on an empty
everything.
avoidancePath > normalPath
*/
AU_UAV_ROS::Command AU_UAV_ROS::PlaneCoordinator::getPriorityCommand()
{
	//start with defaults
	AU_UAV_ROS::Command ret;
	ret.planeID = -1;
	ret.latitude = -1000;
	ret.longitude = -1000;
	ret.altitude = -1000;
	
	//check avoidance queue
	if(!avoidancePath.empty())
	{
		//we have an avoidance point, get that one
		ret.latitude = avoidancePath.front().latitude;
		ret.longitude = avoidancePath.front().longitude;
		ret.altitude = avoidancePath.front().altitude;
	}
	else
	{
		if(!normalPath.empty())
		{
			//we have a normal path point at least, fill it out
			ret.latitude = normalPath.front().latitude;
			ret.longitude = normalPath.front().longitude;
			ret.altitude = normalPath.front().altitude;
		}
		else
		{
			//normal path is also empty do nothing
		}
	}
	
	//fill out our header and return this bad boy
	ret.commandHeader.seq = this->commandIndex++;
	ret.commandHeader.stamp = ros::Time::now();
	return ret;
}	

/*
handleNewUpdate(...)
Takes a received telemetry update and determine if a new command is necessary
*/
bool AU_UAV_ROS::PlaneCoordinator::handleNewUpdate(AU_UAV_ROS::TelemetryUpdate update, AU_UAV_ROS::Command *newCommand)
{
	//this bool is set true only if there is something available to be sent to the UAV
	bool isCommand = false;
	
	//this bool is set if the command available is an avoidance maneuver
	bool isAvoid;
	
	//store the last update
	this->latestUpdate = update;
	
	//data structs to be used by the coordinator
	struct waypoint destination;
	struct waypoint current;
	struct waypoint planeDest;
	
	current.latitude = update.currentLatitude;
	current.longitude = update.currentLongitude;
	current.altitude = update.currentAltitude;

	planeDest.latitude = update.destLatitude;
	planeDest.longitude = update.destLongitude;
	planeDest.altitude = update.destAltitude;
	
	//first se if we need to dump any points from the avoidance path
	while(!avoidancePath.empty() && distanceBetween(avoidancePath.front(), current) < COLLISION_THRESHOLD)
	{
		//this means we met the normal path's first point also, pop it
		avoidancePath.pop_front();
	}
	
	//then see if we need to dump any points from the normal path
	while(!normalPath.empty() && distanceBetween(normalPath.front(), current) < COLLISION_THRESHOLD)
	{
		//this means we met the normal path's first point also, pop it
		normalPath.pop_front();
	}
				
	//determine which point we should be going to right now
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
	
	//if we have a command to process, process it
	if(isCommand)
	{
		//first, check to make sure the plane has the correct current waypoint
		if(distanceBetween(destination, planeDest) > .1 || update.currentWaypointIndex == -1)
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
		
		//if we made it here just return
		return false;
	}
	
	//if we get here, then both queues are empty so just return
	ROS_INFO("Plane #%d has no commands right now.", this->latestUpdate.planeID);
	return false;
}
