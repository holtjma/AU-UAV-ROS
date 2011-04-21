/*
PlaneCoordinator
Class responsible for storing:
A) The latest update from the plane
B) The waypoints to go to (aka path)
C) Any collision avoidance waypoints
*/

#include <stdio.h>

#include "AU_UAV_ROS/SimulatedPlane.h"

AU_UAV_ROS::SimulatedPlane::SimulatedPlane(long long int planeID, AU_UAV_ROS::CreateSimulatedPlane::Request &requestFromUser)
{
	//data from inputs
	this->planeID = planeID;
	this->currentLocation.latitude = requestFromUser.startingLatitude;
	this->currentLocation.longitude = requestFromUser.startingLongitude;
	this->currentLocation.altitude = requestFromUser.startingAltitude;
	this->groundSpeed = requestFromUser.startingGroundSpeed;
	this->bearing = requestFromUser.startingBearing;
	
	//defaults all planes to have no current destination and a starting index of -1
	this->currentDest.latitude = 0;
	this->currentDest.longitude = 0;
	this->currentDest.altitude = 0;
	
	this->currentWaypointIndex = -1;
	this->distanceToDestination = 0;
	
	this->updateIndex = 0;
}

bool AU_UAV_ROS::SimulatedPlane::handleNewCommand(AU_UAV_ROS::Command newCommand)
{
	return false;
}

bool AU_UAV_ROS::SimulatedPlane::fillTelemetryUpdate(AU_UAV_ROS::TelemetryUpdate *tUpdate)
{
	//TODO: update position before sending update
	tUpdate->planeID = this->planeID;
	tUpdate->currentLatitude = this->currentLocation.latitude;
	tUpdate->currentLongitude = this->currentLocation.longitude;
	tUpdate->currentAltitude = this->currentLocation.altitude;
	tUpdate->destLatitude = this->currentDest.latitude;
	tUpdate->destLongitude = this->currentDest.longitude;
	tUpdate->destAltitude = this->currentDest.altitude;
	tUpdate->groundSpeed = this->groundSpeed;
	tUpdate->targetBearing = this->bearing;
	tUpdate->currentWaypointIndex = this->currentWaypointIndex;
	tUpdate->distanceToDestination = this->distanceToDestination;
	tUpdate->telemetryHeader.seq = this->updateIndex++;
	tUpdate->telemetryHeader.stamp = ros::Time::now();
	
	return true;
}

