/*
simulator
This simulator will act as a system for students to test a collision avoidance algorithm prior to real 
physical tests.  It will need to mirror the XBeeIO ROS system along with provide realistic data about
an ArduPilot's flight data.  We will need to look at some real data and maybe ArduPilot code in order to
understand how this works.
*/

//Standard C++ headers
#include <sstream>
#include <list>

//ROS headers
#include "ros/ros.h"
#include "AU_UAV_ROS/standardDefs.h"
#include "AU_UAV_ROS/TelemetryUpdate.h"
#include "AU_UAV_ROS/Command.h"
#include "AU_UAV_ROS/RequestPlaneID.h"
#include "AU_UAV_ROS/CreateSimulatedPlane.h"
#include "AU_UAV_ROS/SimulatedPlane.h"

ros::ServiceClient requestPlaneIDClient;

std::list<AU_UAV_ROS::SimulatedPlane> simPlaneList;

//TODO: we need a callback for receiving commands along with the setup in main(...)
void commandCallback(const AU_UAV_ROS::Command::ConstPtr& msg)
{
	ROS_INFO("Received new message: Plane #%d to (%f, %f, %f)", msg->planeID, msg->latitude, msg->longitude, msg->altitude);
	/*
	waypoint[0] = msg->latitude;
	waypoint[1] = msg->longitude;
	waypoint[2] = msg->altitude;
	*/
}

bool createSimulatedPlane(AU_UAV_ROS::CreateSimulatedPlane::Request &req, AU_UAV_ROS::CreateSimulatedPlane::Response &res)
{
	AU_UAV_ROS::RequestPlaneID srv;
	
	//check to make sure the client call worked (regardless of return values from service)
	if(requestPlaneIDClient.call(srv))
	{
		res.planeID = srv.response.planeID;
		return true;
	}
	else
	{
		ROS_ERROR("Did not receive response from coordinator");
		return false;
	}
}

int main(int argc, char **argv)
{
	//Standard ROS startup
	ros::init(argc, argv, "simulator");
	ros::NodeHandle n;
	
	//setup subscribing to command messages
	ros::Subscriber sub = n.subscribe("commands", 1000, commandCallback);
	
	//setup publishing to telemetry message
	ros::Publisher telemetryPub = n.advertise<AU_UAV_ROS::TelemetryUpdate>("telemetry", 1000);
	
	//setup server services
	ros::ServiceServer createSimulatedPlaneService = n.advertiseService("create_simulated_plane", createSimulatedPlane);
	
	//setup client services
	requestPlaneIDClient = n.serviceClient<AU_UAV_ROS::RequestPlaneID>("request_plane_ID");
	
	//TODO:check for validity of 1 Hz
	//currently updates at 1 Hz, based of Justin Paladino'sestimate of approximately 1 update/sec
	ros::Rate loop_rate(1);
	 
	//keeps count of number of messages sent
	int count = 0;
	
	//while the user doesn't kill the process or we get some crazy error
	while(ros::ok())
	{
		//create an update
		/*AU_UAV_ROS::TelemetryUpdate tUpdate;
		
		//TODO: make this message construction simulated telemetry data
		tUpdate.planeID = 0;
		tUpdate.currentLatitude = 0;
		tUpdate.currentLongitude = 0;
		tUpdate.currentAltitude = 0;
		tUpdate.destLatitude = waypoint[0];
		tUpdate.destLongitude = waypoint[1];
		tUpdate.destAltitude = waypoint[2];
		tUpdate.groundSpeed = 0;
		tUpdate.targetBearing = 0;
		tUpdate.currentWaypointIndex = count;
		tUpdate.distanceToDestination = 0;
		ROS_INFO("Posting update %d", tUpdate.currentWaypointIndex);
		
		
		//publish the message
		telemetryPub.publish(tUpdate);
		*/
		//check for any incoming callbacks and sleep until next update
		ros::spinOnce();
		loop_rate.sleep();
		
		//increment count
		count++;
	}
	
	return 0;
}
