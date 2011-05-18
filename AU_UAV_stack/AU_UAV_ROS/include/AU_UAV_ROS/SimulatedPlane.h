/*
SimulatedPlane
This class is the data structures and functions required to perform plane simulation.  Each one
instantiated will be considered one "plane" in the system.
*/

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
		double groundSpeed;
		double bearing;
		long long int currentWaypointIndex;
		double distanceToDestination;
		
		//index of sent message
		int updateIndex;
		
	public:
		SimulatedPlane();
		SimulatedPlane(long long int planeID, AU_UAV_ROS::CreateSimulatedPlane::Request &requestFromUser);
	
		bool handleNewCommand(AU_UAV_ROS::Command newCommand);
		bool fillTelemetryUpdate(AU_UAV_ROS::TelemetryUpdate *tUpdate);
	};
}
