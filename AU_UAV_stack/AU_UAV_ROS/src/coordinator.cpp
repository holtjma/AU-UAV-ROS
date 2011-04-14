/*
coordinator
This will be responsible for storing plane data for both real and simulated planes (will not be able to tell
a difference here) and for sending commands to those planes.  It will receive commands from collision 
avoidance as well.

TODO: is this where we want to take normal flight commands/read a flight plan?
TODO: I believe this also assigns numbers to planes for messaging purposes
*/

//Standard C++ headers
#include <sstream>

//ROS headers
#include "ros/ros.h"
#include "AU_UAV_ROS/TelemetryUpdate.h"
#include "AU_UAV_ROS/Command.h"
#include "AU_UAV_ROS/AvoidCollision.h"
#include "AU_UAV_ROS/RequestPlaneID.h"

//class headers
#include "AU_UAV_ROS/PlaneCoordinator.h"

//publisher is global so callbacks can access it
ros::Publisher commandPub;

AU_UAV_ROS::PlaneCoordinator planesArray[100];
int numPlanes = 0;

//TODO: this executable should also be sending updates to our commands, normal and avoidance commands

//This function is run whenever a new telemetry update from any plane is recieved
void telemetryCallback(const AU_UAV_ROS::TelemetryUpdate::ConstPtr& msg)
{
	//TODO: Make this function do something useful
	ROS_INFO("Received update #[%lld]", msg->currentWaypointIndex);
	if(msg->planeID < numPlanes)
	{
		//we have a valid plane ID
		planesArray[msg->planeID].handleNewUpdate(*msg);
		
		/*remnants of old test, may be useful in a bit
		if(waypoint[0] != msg->destLatitude || waypoint[1] != msg->destLongitude || waypoint[2] != msg->destAltitude)
		{
			AU_UAV_ROS::Command newCommand;
			newCommand.planeID = 0;
			newCommand.latitude = waypoint[0];
			newCommand.longitude = waypoint[1];
			newCommand.altitude = waypoint[2];
			commandPub.publish(newCommand);
			ROS_INFO("Sent command to plane #%d: (%f, %f, %f)", newCommand.planeID, newCommand.latitude, newCommand.longitude, newCommand.altitude);
		}*/
	}
	else
	{
		ROS_ERROR("Received update from invalid plane ID #%d", msg->planeID);
	}
}

//service to be run whenever the collision avoidance algorithm decides to make a path change
bool avoidCollision(AU_UAV_ROS::AvoidCollision::Request &req, AU_UAV_ROS::AvoidCollision::Response &res)
{
	//TODO: Make this function do something useful
	ROS_INFO("Service Request Recieved: [%s]", req.newCommand.c_str());
	return true;
}

//service to run whenever a new plane enters the arena to tell it the ID number it should use
bool requestPlaneID(AU_UAV_ROS::RequestPlaneID::Request &req, AU_UAV_ROS::RequestPlaneID::Response &res)
{
	//TODO: Set up any data structures related to a plane that need to be created
	if(numPlanes < 100)
	{
		//anything that needs to happen when a plane is instantiated goes here
		res.planeID = numPlanes++;
		return true;
	}
	else
	{
		ROS_ERROR("Too many plane IDs for coordinator to handle.\n");
		return false;
	}
}

int main(int argc, char **argv)
{
	//Standard ROS startup
	ros::init(argc, argv, "coordinator");
	ros::NodeHandle n;
	
	//Subscribe to telemetry message and advertise avoid collision service
	ros::Subscriber sub = n.subscribe("telemetry", 1000, telemetryCallback);
	ros::ServiceServer service = n.advertiseService("avoid_collision", avoidCollision);
	ros::ServiceServer newPlaneServer = n.advertiseService("request_plane_ID", requestPlaneID);
	commandPub = n.advertise<AU_UAV_ROS::Command>("commands", 1000);

	//Needed for ROS to wait for callbacks
	ros::spin();	

	return 0;
}
