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
#include "AU_UAV_ROS/AvoidCollision.h"

//TODO: this executable should also be sending updates to our commands, normal and avoidance commands

//This function is run whenever a new telemetry update from any plane is recieved
void telemetryCallback(const AU_UAV_ROS::TelemetryUpdate::ConstPtr& msg)
{
	//TODO: Make this function do something useful
	ROS_INFO("Received update #[%d]", msg->currentWaypointIndex);
}

//service to be run whenever the collision avoidance algorithm decides to make a path change
bool avoidCollision(AU_UAV_ROS::AvoidCollision::Request &req, AU_UAV_ROS::AvoidCollision::Response &res)
{
	//TODO: Make this function do something useful
	ROS_INFO("Service Request Recieved: [%s]", req.newCommand.c_str());
	return true;
}

int main(int argc, char **argv)
{
	//Standard ROS startup
	ros::init(argc, argv, "coordinator");
	ros::NodeHandle n;
	
	//Subscribe to telemetry message and advertise avoid collision service
	ros::Subscriber sub = n.subscribe("telemetry", 1000, telemetryCallback);
	ros::ServiceServer service = n.advertiseService("avoid_collision", avoidCollision);

	//Needed for ROS to wait for callbacks
	ros::spin();	

	return 0;
}
