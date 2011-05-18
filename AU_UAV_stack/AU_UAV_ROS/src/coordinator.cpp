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
//#include "AU_UAV_ROS/AvoidCollision.h"
#include "AU_UAV_ROS/RequestPlaneID.h"
#include "AU_UAV_ROS/GoToWaypoint.h"

//class headers
#include "AU_UAV_ROS/standardDefs.h"
#include "AU_UAV_ROS/PlaneCoordinator.h"

//publisher is global so callbacks can access it
ros::Publisher commandPub;

AU_UAV_ROS::PlaneCoordinator planesArray[100];
int numPlanes = 0;

//TODO: this executable should also be sending updates to our commands, normal and avoidance commands

bool isValidPlaneID(int id)
{
	if(id >= 0 && id < numPlanes) return true;
	else return false;
}

//This function is run whenever a new telemetry update from any plane is recieved
void telemetryCallback(const AU_UAV_ROS::TelemetryUpdate::ConstPtr& msg)
{
	//TODO: Make this function do something useful
	ROS_INFO("Received update #[%d] from plane ID %d", msg->telemetryHeader.seq, msg->planeID);
	if(isValidPlaneID(msg->planeID))
	{
		AU_UAV_ROS::Command commandToSend;
		//we have a valid plane ID
		if(planesArray[msg->planeID].handleNewUpdate(*msg, &commandToSend))
		{
			//send new command
			commandPub.publish(commandToSend);
			ROS_INFO("Sent command to plane #%d: (%f, %f, %f)", commandToSend.planeID, commandToSend.latitude, commandToSend.longitude, commandToSend.altitude);
		}
		else
		{
			//don't send a new command
		}
		
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

//NOTE: This is a remnant from an old design, use go to waypoint with the right modifiers
//service to be run whenever the collision avoidance algorithm decides to make a path change
/*bool avoidCollision(AU_UAV_ROS::AvoidCollision::Request &req, AU_UAV_ROS::AvoidCollision::Response &res)
{
	//TODO: Make this function do something useful
	ROS_INFO("Service Request Received: [%s]", req.newCommand.c_str());
	return true;
}*/

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

//service to run whenever the simple control menu requests for a plane to go to a particular point
bool goToWaypoint(AU_UAV_ROS::GoToWaypoint::Request &req, AU_UAV_ROS::GoToWaypoint::Response &res)
{
	ROS_INFO("Service Request Received: Plane #%d go to (%f, %f, %f)", req.planeID, req.latitude, req.longitude, req.altitude);	
	
	//check for valid plane ID
	if(isValidPlaneID(req.planeID))
	{
		//construct waypoint
		struct AU_UAV_ROS::waypoint pointFromService;
		pointFromService.latitude = req.latitude;
		pointFromService.longitude = req.longitude;
		pointFromService.altitude = req.altitude;
		
		//attempt to set waypoint
		if(planesArray[req.planeID].goToPoint(pointFromService, req.isAvoidanceManeuver, req.isNewQueue))
		{
			//success!
			return true;
		}
		else
		{
			//this should never happen in the current setup
			ROS_ERROR("Error in planesArray[%d].goToPoint(...)", req.planeID);
			return false;
		}
	}
	else
	{
		ROS_ERROR("Invalid plane ID in GoToWaypoint service request.\n");
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
	//ros::ServiceServer avoidCollisionServer = n.advertiseService("avoid_collision", avoidCollision);
	ros::ServiceServer newPlaneServer = n.advertiseService("request_plane_ID", requestPlaneID);
	ros::ServiceServer goToWaypointServer = n.advertiseService("go_to_waypoint", goToWaypoint);
	commandPub = n.advertise<AU_UAV_ROS::Command>("commands", 1000);

	//Needed for ROS to wait for callbacks
	ros::spin();	

	return 0;
}
