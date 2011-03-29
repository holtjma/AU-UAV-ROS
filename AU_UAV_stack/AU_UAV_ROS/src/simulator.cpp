/*
simulator
This simulator will act as a system for students to test a collision avoidance algorithm prior to real 
physical tests.  It will need to mirror the XBeeIO ROS system along with provide realistic data about
an ArduPilot's flight data.  We will need to look at some real data and maybe ArduPilot code in order to
understand how this works.
*/

//Standard C++ headers
#include <sstream>

//ROS headers
#include "ros/ros.h"
#include "AU_UAV_ROS/TelemetryUpdate.h"

//TODO: we need a callback for receiving commands along with the setup in main(...)

int main(int argc, char **argv)
{
	//Standard ROS startup
	ros::init(argc, argv, "simulator");
	ros::NodeHandle n;
	
	//setup publishing to telemetry message
	ros::Publisher telemetryPub = n.advertise<AU_UAV_ROS::TelemetryUpdate>("telemetry", 1000);
	
	//TODO:check for validity of 1 Hz
	//currently updates at 1 Hz, based of Justin Paladino'sestimate of approximately 1 update/sec
	ros::Rate loop_rate(1);
	 
	//keeps count of number of messages sent
	int count = 0;
	
	//while the user doesn't kill the process or we get some crazy error
	while(ros::ok())
	{
		//create an update
		AU_UAV_ROS::TelemetryUpdate tUpdate;
		
		//TODO: make this message construction simulated telemetry data
		//set update string
		
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
		ss << "This is a string from sim:" << count;
		tUpdate.update = ss.str();
		ROS_INFO("%s", tUpdate.update.c_str());*/
		
		//publish the message
		telemetryPub.publish(tUpdate);
		
		//check for any incoming callbacks and sleep until next update
		ros::spinOnce();
		loop_rate.sleep();
		
		//increment count
		count++;
	}
	
	return 0;
}
