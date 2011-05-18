/*
PlaneCoordinator
Class responsible for storing:
A) The latest update from the plane
B) The waypoints to go to (aka path)
C) Any collision avoidance waypoints
*/

#include <stdio.h>
#include <math.h>

#include "AU_UAV_ROS/SimulatedPlane.h"

//25 mph = 11.17600 meters / second
#define MPS_SPEED 11.176 
#define MPH_SPEED 25

//meters
#define EARTH_RADIUS 6371000
#define DEGREES_TO_RADIANS (M_PI/180.0)
#define RADIANS_TO_DEGREES (180.0/M_PI)

AU_UAV_ROS::SimulatedPlane::SimulatedPlane()
{
	//make a bad ID
	this->planeID = -1;
}

AU_UAV_ROS::SimulatedPlane::SimulatedPlane(long long int planeID, AU_UAV_ROS::CreateSimulatedPlane::Request &requestFromUser)
{
	//data from inputs
	this->planeID = planeID;
	this->currentLocation.latitude = requestFromUser.startingLatitude;
	this->currentLocation.longitude = requestFromUser.startingLongitude;
	this->currentLocation.altitude = requestFromUser.startingAltitude;
	this->groundSpeed = MPH_SPEED;
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
	ROS_INFO("Handling new command for plane ID #%lld", this->planeID);
	//check to make sure we're in the right place
	if(this->planeID != newCommand.planeID)
	{
		//this should NEVER happen
		return false;
	}
	
	//set our destination to the command
	this->currentDest.latitude = newCommand.latitude;
	this->currentDest.longitude = newCommand.longitude;
	this->currentDest.altitude = newCommand.altitude;
	
	//I think we just increment this waypoint index?
	this->currentWaypointIndex++;
	
	//return success
	return true;
}

/*
fillTelemetryUpdate
This function is based on a number of assumptions to simulate the next point:
1. One second has passed since we are performing 1 Hz updates
2. The 'haversine' formula is used for distance between lats/longs: http://en.wikipedia.org/wiki/Haversine_formula
3. We assume perfect world scenario (no friction, wind, etc)
4. We used an estimated 25 mph for the speed

Subject to change (aka improvement) assumptions:
5. the plane goes directly from one point to another (no turns)
*/
bool AU_UAV_ROS::SimulatedPlane::fillTelemetryUpdate(AU_UAV_ROS::TelemetryUpdate *tUpdate)
{
	//difference in latitudes in radians
	double lat1 = currentLocation.latitude*DEGREES_TO_RADIANS;
	double lat2 = currentDest.latitude*DEGREES_TO_RADIANS;
	double long1 = currentLocation.longitude*DEGREES_TO_RADIANS;
	double long2 = currentDest.longitude*DEGREES_TO_RADIANS;
	
	double deltaLat = lat2 - lat1;
	double deltaLong = long2 - long1;
	
	//haversine crazy math, should probably be verified further beyond basic testing
	double a = pow(sin(deltaLat / 2.0), 2);
	a = a + cos(lat1)*cos(lat2)*pow(sin(deltaLong/2.0), 2);
	double c = 2.0 * asin(sqrt(a));
	this->distanceToDestination = EARTH_RADIUS * c;
	
	//calculate bearing
	double y = sin(deltaLong)*cos(lat2);
	double x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(deltaLong);
	this->bearing = atan2(y, x)*RADIANS_TO_DEGREES;

	//fill out the actual data
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

