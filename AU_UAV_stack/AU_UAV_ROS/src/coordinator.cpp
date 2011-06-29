/*
coordinator
This will be responsible for storing plane data for both real and simulated planes (will not be able to tell
a difference here) and for sending commands to those planes.  It will receive commands from collision 
avoidance as well.
*/

//Standard C++ headers
#include <sstream>

//ROS headers
#include "ros/ros.h"
#include "ros/package.h"
#include "AU_UAV_ROS/TelemetryUpdate.h"
#include "AU_UAV_ROS/Command.h"
#include "AU_UAV_ROS/RequestPlaneID.h"
#include "AU_UAV_ROS/GoToWaypoint.h"
#include "AU_UAV_ROS/PlaneCoordinator.h"
#include "AU_UAV_ROS/LoadPath.h"
#include "AU_UAV_ROS/RequestWaypointInfo.h"
#include "AU_UAV_ROS/LoadCourse.h"

//publisher is global so callbacks can access it
ros::Publisher commandPub;

//coordinator list of UAVs, may want to lengthen this or perhaps change it to a map, not sure
std::map<int, AU_UAV_ROS::PlaneCoordinator> planesArray;

//just a count of the number of planes so far, init to zero
int numPlanes = 0;

/*
isValidPlanID(...)
simple function to make sure that an ID sent to us is known by the coordinator
*/
bool isValidPlaneID(int id)
{
	//if the number id is valid
	//AND we have that id in the map
	//AND that UAV is active
	if(id >= 0 && planesArray.find(id) != planesArray.end() && planesArray[id].isActive) return true;
	else return false;
}

/*
telemetryCallback(...)
This function is run whenever a new telemetry update from any plane is recieved.  Mainly, it forwards the
update onwards and it will send new commands if the plane coordinators deem it necessary.
*/
void telemetryCallback(const AU_UAV_ROS::TelemetryUpdate::ConstPtr& msg)
{
	ROS_INFO("Received update #[%d] from plane ID %d", msg->telemetryHeader.seq, msg->planeID);
	
	//check the make sure the update is valid first
	if(isValidPlaneID(msg->planeID))
	{
		//prep in case a command needs to be sent
		AU_UAV_ROS::Command commandToSend;
		
		//check whether the update warrants a new command or not
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
	}
	else
	{
		ROS_ERROR("Received update from invalid plane ID #%d", msg->planeID);
	}
}

//service to run whenever a new plane enters the arena to tell it the ID number it should use
bool requestPlaneID(AU_UAV_ROS::RequestPlaneID::Request &req, AU_UAV_ROS::RequestPlaneID::Response &res)
{
	//check to see if we've been given an ID
	if(req.requestedID == -1)
	{
		//we weren't given an ID, so pick the first that's free
		int id = 0;
		while(true)
		{
			//check if the ID is occupied or inactive
			if(planesArray.find(id) != planesArray.end() && planesArray[id].isActive)
			{
				//this id already exists, increment our id and try again
				id++;
			}
			else
			{
				//we found an unused ID, lets steal it
				planesArray[id] = AU_UAV_ROS::PlaneCoordinator();
				planesArray[id].isActive = true;
				numPlanes++;
				
				res.planeID = id;
				return true;
			}
		}
		
		//not sure how we'd get here, but better to handle it
		return false;
	}
	else
	{
		//we've been given an ID, check if it's open or inactive
		if(planesArray.find(req.requestedID) == planesArray.end() || !planesArray[req.requestedID].isActive)
		{
			planesArray[req.requestedID] = AU_UAV_ROS::PlaneCoordinator();
			planesArray[req.requestedID].isActive = true;
			numPlanes++;
			res.planeID = req.requestedID;
			return true;
		}
		else
		{
			ROS_ERROR("The ID #%d is already taken.\n", req.requestedID);
			return false;
		}
	}
}

/*
goToWaypoint(...)
This is the key waypoint function to be used by any external program for telling the coordinator to add
something to our queue.  The request gives it a point, a queue, and whether the queue should be erased before
this new point is added.
*/
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
			
			/*
			if we have a new queue, we want to forward the new point immediately, otherwise just
			let the normal command sending do it's work
			*/
			if(req.isNewQueue)
			{
				//get the command
				AU_UAV_ROS::Command commandToSend = planesArray[req.planeID].getPriorityCommand();
				commandToSend.planeID = req.planeID;
				
				if(commandToSend.latitude == -1000 || commandToSend.longitude == -1000 || commandToSend.altitude == -1000)
				{
					//dont send it
				}
				else
				{
					commandPub.publish(commandToSend);
					ROS_INFO("Sent command to plane #%d: (%f, %f, %f)", commandToSend.planeID, commandToSend.latitude, commandToSend.longitude, commandToSend.altitude);
				}
				
			}
			
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

/*
loadPathCallback(...)
This is the callback used when the user requests for a plane to start a certain path.
*/
bool loadPathCallback(AU_UAV_ROS::LoadPath::Request &req, AU_UAV_ROS::LoadPath::Response &res)
{
	ROS_INFO("Received request: Load path from \"%s\" to plane #%d\n", req.filename.c_str(), req.planeID);
	
	//check for a valid plane ID sent
	if(isValidPlaneID(req.planeID))
	{
		//open our file
		FILE *fp;
		fp = fopen((ros::package::getPath("AU_UAV_ROS")+"/paths/"+req.filename).c_str(), "r");
		
		//check for a good file open
		if(fp != NULL)
		{
			char buffer[256];
			bool isFirstLine = true;
			bool isAvoidance = false;
			
			//while we have something in the file
			while(fgets(buffer, sizeof(buffer), fp))
			{
				//check first character for comment or if the line is blank
				if(buffer[0] == '#' || isBlankLine(buffer))
				{
					//this line is a comment
					continue;
				}
				else
				{
					//set some invalid defaults
					struct AU_UAV_ROS::waypoint temp;
					temp.latitude = temp.longitude = temp.altitude = -1000;
					
					//parse the string
					sscanf(buffer, "%lf %lf %lf\n", &temp.latitude, &temp.longitude, &temp.altitude);
					ROS_INFO("%lf %lf %lf", temp.latitude, temp.longitude, temp.altitude);
					//check for the invalid defaults
					if(temp.latitude == -1000 || temp.longitude == -1000 || temp.altitude == -1000)
					{
						//this means we have a bad file somehow
						ROS_ERROR("Bad file parse");
						res.error = "Bad file parse, some points loaded";
						return false;
					}
					
					//call the goToPoint function for the correct plane
					planesArray[req.planeID].goToPoint(temp, isAvoidance, isFirstLine);
					
					//only clear the queue with the first point
					if(isFirstLine) isFirstLine = false;
				}
			}
			
			//end of file, return
			return true;
		}
		else
		{
			ROS_ERROR("Invalid filename or location: %s", req.filename.c_str());
			res.error = "Invalid filename or location";
			return false;
		}
	}
	else
	{
		ROS_ERROR("Invalid plane ID");	
		res.error = "Invalid plane ID";
		return false;
	}
}

/*
loadCourseCallback(...)
This callback takes a filename and parses that file to load a course into the coordinator. A course in
this sense would be several paths for multiple UAVs.  We want this so we can test a collision avoidance
course by preplanning the course.
*/
bool loadCourseCallback(AU_UAV_ROS::LoadCourse::Request &req, AU_UAV_ROS::LoadCourse::Response &res)
{
	ROS_INFO("Received request: Load course from \"%s\"\n", req.filename.c_str());
	
	//open our file
	FILE *fp;
	fp = fopen((ros::package::getPath("AU_UAV_ROS")+"/courses/"+req.filename).c_str(), "r");
	
	//check for a good file open
	if(fp != NULL)
	{
		char buffer[256];
		
		std::map<int, bool> isFirstPoint;
		bool isAvoidance = false;
		
		//while we have something in the file
		while(fgets(buffer, sizeof(buffer), fp))
		{
			//check first character
			if(buffer[0] == '#' || isBlankLine(buffer))
			{
				//this line is a comment
				continue;
			}
			else
			{
				//set some invalid defaults
				int planeID = -1;
				struct AU_UAV_ROS::waypoint temp;
				temp.latitude = temp.longitude = temp.altitude = -1000;
				
				//TODO: change the parser to create planes that don't exist yet?
				//parse the string
				sscanf(buffer, "%d %lf %lf %lf\n", &planeID, &temp.latitude, &temp.longitude, &temp.altitude);
				ROS_INFO("Loaded %d to %lf %lf %lf", planeID, temp.latitude, temp.longitude, temp.altitude);
				
				//check for the invalid defaults
				if(planeID == -1 || temp.latitude == -1000 || temp.longitude == -1000 || temp.altitude == -1000)
				{
					//this means we have a bad file somehow
					ROS_ERROR("Bad file parse");
					res.error = "Bad file parse, some points loaded";
					return false;
				}
				
				//check our map for an entry, if we dont have one then this is the first time
				//that this plane ID has been referenced so it's true
				if(isFirstPoint.find(planeID) == isFirstPoint.end())
				{
					isFirstPoint[planeID] = true;
				}
				
				//call the goToPoint function for the correct plane
				planesArray[planeID].goToPoint(temp, isAvoidance, isFirstPoint[planeID]);
				
				//only clear the queue with the first point
				if(isFirstPoint[planeID]) isFirstPoint[planeID] = false;
			}
		}
		
		//end of file, return
		return true;
	}
	else
	{
		ROS_ERROR("Invalid filename or location: %s", req.filename.c_str());
		res.error = "Invalid filename or location";
		return false;
	}
}

/*
requestWaypointInfoCallback(...)
This callback allows the avoidance algorithms (or I suppose anything really) to view the waypoint queues
in the system.  It takes a plane ID, a boolean value that reflects whether it's a request for the avoidance
queue or the normal queue, and finally a position in that queue.  If there isn't a value at the requested
place, we fill in the error message and return a point (-1000, -1000, -1000) simply because it's not a valid
position.

@position - follows normal array standards, aka 0 is the front of the queue
*/
bool requestWaypointInfoCallback(AU_UAV_ROS::RequestWaypointInfo::Request &req, AU_UAV_ROS::RequestWaypointInfo::Response &res)
{
	//check that the request ID is valid
	if(isValidPlaneID(req.planeID))
	{
		//fill out the waypoint to return
		AU_UAV_ROS::waypoint temp = planesArray[req.planeID].getWaypointOfQueue(req.isAvoidanceWaypoint, req.positionInQueue);
		res.latitude = temp.latitude;
		res.longitude = temp.longitude;
		res.altitude = temp.altitude;
		
		//check if we need to return the error message
		if(res.latitude == -1000 && res.longitude == -1000 && res.altitude == -1000)
		{
			res.error = "No points in that queue";
		}
		
		return true;
	}
	else
	{
		ROS_ERROR("Invalid plane ID");
		res.error = "Invalid plane ID";
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
	ros::ServiceServer newPlaneServer = n.advertiseService("request_plane_ID", requestPlaneID);
	ros::ServiceServer goToWaypointServer = n.advertiseService("go_to_waypoint", goToWaypoint);
	ros::ServiceServer loadPathServer = n.advertiseService("load_path", loadPathCallback);
	ros::ServiceServer requestWaypointInfo = n.advertiseService("request_waypoint_info", requestWaypointInfoCallback);
	ros::ServiceServer loadCourseServer = n.advertiseService("load_course", loadCourseCallback);
	commandPub = n.advertise<AU_UAV_ROS::Command>("commands", 1000);

	//Needed for ROS to wait for callbacks
	ros::spin();	

	return 0;
}
