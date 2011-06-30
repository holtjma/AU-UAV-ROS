/*
PlaneCoordinator
Class responsible for storing:
A) The latest update from the plane
B) The waypoints to go to (aka path)
C) Any collision avoidance waypoints
*/

//basic headers
#include <stdio.h>
#include <math.h>

//ROS headers
#include "AU_UAV_ROS/standardDefs.h"
#include "AU_UAV_ROS/SimulatedPlane.h"

//standard constructor, shouldn't really be used
AU_UAV_ROS::SimulatedPlane::SimulatedPlane()
{
	//make a bad ID
	this->planeID = -1;
}

/*
SimulatedPlane(...)
This is the primary constructor to be used.  Note how all information is extracted from the service request
for creation of a simulated plane.
*/
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
	
	//set this to be the bearing requested, typically 0 right now though
	this->actualBearing = requestFromUser.startingBearing;
	
	this->updateIndex = 0;
}

/*
handleNewCommand(...)
This function takes a command from the coordinator and stores the new information for simulation later.
That's really all it does, no calculations or anything.
*/
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
	//calculate distance from current position to destination
	double a = pow(sin(deltaLat / 2.0), 2);
	a = a + cos(lat1)*cos(lat2)*pow(sin(deltaLong/2.0), 2);
	double c = 2.0 * asin(sqrt(a));
	this->distanceToDestination = EARTH_RADIUS * c;
	
	//calculate bearing from current position to destination
	double y = sin(deltaLong)*cos(lat2);
	double x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(deltaLong);
	this->bearing = atan2(y, x)*RADIANS_TO_DEGREES;
	
	//make sure we're actually traveling somewhere
	if(this->currentWaypointIndex >= 0)
	{
		//calculate the real bearing based on our maximum angle change
		//first create a temporary bearing that is the same as bearing but at a different numerical value
		double tempBearing = -1000;
		if((this->bearing) < 0)
		{
			tempBearing = this->bearing + 360;
		}
		else
		{
			tempBearing = this->bearing - 360;
		}
		
		double diff1 = abs(this->actualBearing - this->bearing);
		double diff2 = abs(this->actualBearing - tempBearing);
	
		//check for easy to calculate values first
		if(diff1 < MAXIMUM_TURNING_ANGLE || diff2 < MAXIMUM_TURNING_ANGLE)
		{
			//the difference is less than our maximum angle, set it to the bearing
			this->actualBearing = this->bearing;
		}
		else
		{
			//we have a larger difference than we can turn, so turn our maximum
			double mod;
			if(diff1 < diff2)
			{
				if(this->bearing > this->actualBearing) mod = MAXIMUM_TURNING_ANGLE;
				else mod = 0 - MAXIMUM_TURNING_ANGLE;
			}
			else
			{
				if(tempBearing > this->actualBearing) mod = MAXIMUM_TURNING_ANGLE;
				else mod = 0 - MAXIMUM_TURNING_ANGLE;
			}
		
			//add our mod, either +22.5 or -22.5
			this->actualBearing = this->actualBearing + mod;
		
			//tweak the value to keep it between -180 and 180
			if(this->actualBearing > 180) this->actualBearing = this->actualBearing - 360;
			if(this->actualBearing <= -180) this->actualBearing = this->actualBearing + 360;
		}
	
		//time to calculate the new positions, God help us
		/*
		Algorithm for updating position:
		1) Estimate new latitude using basic trig and this equation:
		   lat2 = lat1 + (MPS_SPEED*cos(bearing))*METERS_TO_LATITUDE
		2) Use law of haversines to find the new longitude
		   haversin(c) = haversin(a-b) + sin(a)*sin(b)*haversin(C)
		   where haversin(x) = (sin(x/2.0))^2
		   where c = MPS_SPEED/EARTH_RADIUS (radians)
		   where a = 90 - lat1 (degrees)
		   where b = 90 - lat2 (degrees)
		   where C = the change in longitude, what we are solving for
		   
		   C = 2.0 * arcsin(sqrt((haversin(c) - haversin(a-b))/(sin(a)*sin(b))))
		*/
	
		//1) Estimate new latitude using basic trig and this equation
		this->currentLocation.latitude = lat1*RADIANS_TO_DEGREES + (MPS_SPEED*cos(this->actualBearing*DEGREES_TO_RADIANS))*METERS_TO_LATITUDE;
		
		//2) Use the law of haversines to find the new longitude
		//double temp = pow(sin((MPS_SPEED/EARTH_RADIUS)/2.0), 2);
		double temp = 7.69303281*pow(10, -13); //always the same, see above calculation
		temp = temp - pow(sin((this->currentLocation.latitude*DEGREES_TO_RADIANS - lat1)/2.0), 2);
		temp = temp / (sin(M_PI/2.0 - lat1)*sin((M_PI/2.0)-this->currentLocation.latitude*DEGREES_TO_RADIANS));
		temp = 2.0 * RADIANS_TO_DEGREES * asin(sqrt(temp));
		
		//depending on bearing, we should be either gaining or losing longitude
		if(actualBearing > 0)
		{
			this->currentLocation.longitude += temp;
		}
		else
		{
			this->currentLocation.longitude -= temp;
		}
	}
		
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

