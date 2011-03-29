/*
XBeeIO
The purpose of XBeeIO is to act as a ROS bridge between the XBee communication system and the ROS network.
We will need to port XBee code into here somewhere in order to communicate over the XBee network.
*/

//Standard C++ headers
#include <sstream>

//ROS headers
#include "ros/ros.h"
#include "AU_UAV_ROS/TelemetryUpdate.h"

//TODO: this executable should have a callback for when a new command, normal or avoidance, is received
//TODO: this executable should have a callback for when messages come in over XBee

int main(int argc, char **argv) 
{
	//normal ROS startup
	ros::init(argc, argv, "XBeeIO");
	ros::NodeHandle n;
	
	//Advertise telemetry updates
	ros::Publisher telemetryPub = n.advertise<AU_UAV_ROS::TelemetryUpdate>("telemetry", 1000);
	
	//TODO:this needs to be removed.  When real comm. is active, there will be triggers since this acts
	//as a bridge
	ros::Rate loop_rate(1);
	
	//keep count, not really necessary
	int count = 0;
	
	//while user doesn't kill process and all things are normal
	while(ros::ok())
	{
		//construct a telemetry update
		AU_UAV_ROS::TelemetryUpdate tUpdate;
		
		//Construct a dummy message for now
		tUpdate.planeID = 0;
		tUpdate.currentLatitude = 0;
		tUpdate.currentLongitude = 0;
		tUpdate.currentAltitude = 0;
		tUpdate.destLatitude = 0;
		tUpdate.destLongitude = 0;
		tUpdate.destAltitude = 0;
		tUpdate.groundSpeed = 0;
		tUpdate.targetBearing = 0;
		tUpdate.currentWaypointIndex = count;
		tUpdate.distanceToDestination = 0;
		ROS_INFO("Posting update %d", tUpdate.currentWaypointIndex);
		
		/*std::stringstream ss;
		ss << "This is a string from XbeeIO:" << count;
		tUpdate.update = ss.str();
		ROS_INFO("%s", tUpdate.update.c_str());*/
		
		//publish the message
		telemetryPub.publish(tUpdate);
		
		//check for callbacks and then sleep
		ros::spinOnce();
		loop_rate.sleep();
		
		//increment count
		count++;
	}
	
	return 0;
}
