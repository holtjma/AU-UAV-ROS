/*
SimulatedPlane
This class is the data structures and functions required to perform plane simulation.  Each one
instantiated will be considered one "plane" in the system.
*/

#ifndef SIMULATED_PLANE_H
#define SIMULATED_PLANE_H

#define MAXIMUM_TURNING_ANGLE 22.5 //degrees

#include "AU_UAV_ROS/standardDefs.h"
#include "AU_UAV_ROS/Command.h"
#include "AU_UAV_ROS/TelemetryUpdate.h"
#include "AU_UAV_ROS/CreateSimulatedPlane.h"

namespace AU_UAV_ROS
{
	class SimulatedPlane
	{
	private:
		//last received command info
		AU_UAV_ROS::Command lastCommand;
		
		//current information (used mostly in update)
		long long int planeID;
		
		AU_UAV_ROS::waypoint currentLocation;
		AU_UAV_ROS::waypoint currentDest;
		
		//these two values are sent in the telemetry update
		double groundSpeed;
		double bearing;
		
		//this is stored as part of the UAV info
		double actualBearing;
		
		long long int currentWaypointIndex;
		double distanceToDestination;
		
		//index of sent message
		int updateIndex;
		
	public:
		//dummy constructor, shouldn't really be used
		SimulatedPlane();
		
		//primary constructor
		SimulatedPlane(long long int planeID, AU_UAV_ROS::CreateSimulatedPlane::Request &requestFromUser);
	
		//function for handling a command from the coordinator
		bool handleNewCommand(AU_UAV_ROS::Command newCommand);
		
		//periodic function for filling in a new telemetry update for this UAV
		bool fillTelemetryUpdate(AU_UAV_ROS::TelemetryUpdate *tUpdate);
	};
}

#endif
