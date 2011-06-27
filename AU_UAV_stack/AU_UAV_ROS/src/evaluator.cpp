/*
evaluator
This is the end-of-semester REU evaluation program intended to score the system based on a number
of pre-determined variables and to output that information to a file as well as perform logging 
functions on that data.
*/

//standard headers
#include <stdio.h>
#include <stdlib.h>
#include <map>
#include <queue>

//ROS headers
#include "ros/ros.h"
#include "ros/package.h"
#include "AU_UAV_ROS/standardDefs.h"
#include "AU_UAV_ROS/TelemetryUpdate.h"
#include "AU_UAV_ROS/LoadCourse.h"
#include "AU_UAV_ROS/CreateSimulatedPlane.h"
#include "AU_UAV_ROS/DeleteSimulatedPlane.h"

//USER DEFINED EVALUATION SETTINGS
#define TIME_LIMIT 600 //10 minutes
#define WAYPOINT_SCORE 5 //5 points for each waypoint reached
#define CONFLICT_SCORE -3 //-3 points for each conflict during flight

//services to the simulator
ros::ServiceClient createSimulatedPlaneClient;
ros::ServiceClient deleteSimulatedPlaneClient;

//services to the coordinator
ros::ServiceClient loadCourseClient;

//keep all our updates listed here
std::map<int, AU_UAV_ROS::TelemetryUpdate> previousUpdatesMap;
std::map<int, AU_UAV_ROS::TelemetryUpdate> latestUpdatesMap;

//the list of waypoints to go at
std::map<int, std::queue<AU_UAV_ROS::waypoint> > waypointQueues;

//when our program starts
ros::Time startTime;

//data about each plane
std::map<int, bool> isDead;
std::map<int, ros::Duration> timeOfDeath;
std::map<int, double> distanceTraveled;
std::map<int, int> waypointsAchieved;
std::map<int, double> minimumTravelDistance;

//list for deletion
std::queue<int> planesToDelete;

//overall data
int waypointsTotal = 0;
int numConflicts = 0;
int numCollisions = 0;
int deadPlaneCount = 0;
float totalDistTraveled = 0;
float totalMinDist = 0;
int score = 0;

//store the "last" ID
int lastPlaneID = -1;

/*
displayOutput()
Just a consolidated function for updating what's reflected on the screen during a test
*/
void displayOutput()
{
	system("clear");
	
	printf("Plane ID\tDistance Traveled(m)\tMinimum Travel(m)\t\tWaypoints Achieved\tTime of Death(s)\n");
	printf("--------\t--------------------\t-----------------\t\t------------------\t----------------\n");
	for(int id = 0; id <= lastPlaneID; id++)
	{
		printf("%d\t\t%lf\t\t%lf\t\t\t%d\t\t\t", id, distanceTraveled[id], minimumTravelDistance[id], waypointsAchieved[id]);
		if(isDead[id])
		{
			printf("%lf\n", timeOfDeath[id].toSec());
		}
		else
		{
			printf("ALIVE\n");
		}
	}
	printf("\n");
	printf("Totals:\n");
	printf("Waypoints reached: %d\n", waypointsTotal);
	printf("Number of conflicts: %d\n", numConflicts);
	printf("Number of collisison: %d\n", numCollisions);
	printf("Dead plane count: %d\n", deadPlaneCount);
	if(totalDistTraveled != 0) printf("Distance actual/distance minimum: %lf\n", totalDistTraveled/totalMinDist);
	printf("Score: %d\n", score);
}

/*
createCourseUAVs(...)
takes a filename and will parse it to determine how many UAVs there are and create them as needed
NOTE: this was copied from the ControlMenu and modified somewhat
*/
bool createCourseUAVs(std::string filename)
{
	//open our file
	FILE *fp;
	fp = fopen((ros::package::getPath("AU_UAV_ROS")+"/courses/"+filename).c_str(), "r");
	
	//check for a good file open
	if(fp != NULL)
	{
		char buffer[256];
		
		std::map<int, bool> isFirstPoint;
		
		while(fgets(buffer, sizeof(buffer), fp))
		{
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
				
				//parse the string
				sscanf(buffer, "%d %lf %lf %lf\n", &planeID, &temp.latitude, &temp.longitude, &temp.altitude);
				
				//check for the invalid defaults
				if(planeID == -1 || temp.latitude == -1000 || temp.longitude == -1000 || temp.altitude == -1000)
				{
					//this means we have a bad file somehow
					ROS_ERROR("Bad file parse");
					return false;
				}
				
				//add this point to the correct queue
				waypointQueues[planeID].push(temp);
				
				//check our map for an entry, if we dont have one then this is the first time
				//that this plane ID has been referenced so it's true
				if(isFirstPoint.find(planeID) == isFirstPoint.end())
				{
					isFirstPoint[planeID] = true;
					
					//this is the first time we've seen this ID in the file, attempt to create it
					AU_UAV_ROS::CreateSimulatedPlane srv;
					srv.request.startingLatitude = temp.latitude;
					srv.request.startingLongitude = temp.longitude;
					srv.request.startingAltitude = temp.altitude;
					srv.request.startingBearing = 0;
					srv.request.requestedID = planeID;
				
					//send the service request
					printf("\nRequesting to create new plane with ID #%d...\n", planeID);
					if(createSimulatedPlaneClient.call(srv))
					{
						printf("New plane with ID #%d has been created!\n", srv.response.planeID);
					}
					else
					{
						ROS_ERROR("Did not receive a response from simulator");
					}
					
					//set our last plane ID to this one
					lastPlaneID = planeID;
					latestUpdatesMap[planeID] = AU_UAV_ROS::TelemetryUpdate();
					latestUpdatesMap[planeID].currentLatitude = temp.latitude;
					latestUpdatesMap[planeID].currentLongitude = temp.longitude;
					latestUpdatesMap[planeID].currentAltitude = temp.altitude;
					
					previousUpdatesMap[planeID] = AU_UAV_ROS::TelemetryUpdate();
					previousUpdatesMap[planeID].currentLatitude = temp.latitude;
					previousUpdatesMap[planeID].currentLongitude = temp.longitude;
					previousUpdatesMap[planeID].currentAltitude = temp.altitude;
					
					distanceTraveled[planeID] = 0;
					waypointsAchieved[planeID] = 0;
					minimumTravelDistance[planeID] = 0;
					isDead[planeID] = false;
				}
					
				//only clear the queue with the first point
				if(isFirstPoint[planeID]) isFirstPoint[planeID] = false;
			}
		}
		
		return true;
	}
	else
	{
		ROS_ERROR("Invalid filename or location: %s", filename.c_str());
		return false;
	}
}

/*
telemetryCallback
This is called whenever a new telemetry message is received.  We should store this any waypoint info received
and perform analysis on it once all planes have received new data.
*/
void telemetryCallback(const AU_UAV_ROS::TelemetryUpdate::ConstPtr& msg)
{
	//make sure the sim isn't lagging somehow and still reporting dead planes
	if(isDead[msg->planeID]) return;
	
	//store the telemetry information received
	previousUpdatesMap[msg->planeID] = latestUpdatesMap[msg->planeID];
	latestUpdatesMap[msg->planeID] = *msg;
	
	//if all UAVs have an update, run some analysis on this timestep
	if(msg->planeID == lastPlaneID)
	{
		struct AU_UAV_ROS::waypoint current, other;
		//perform calculations on each plane
		for(int id = 0; id <= lastPlaneID; id++)
		{
			//nothing to do... WHEN YOU'RE DEAD!
			if(isDead[id]) continue;
			
			//change to waypoint format
			other.latitude = previousUpdatesMap[id].currentLatitude;
			other.longitude = previousUpdatesMap[id].currentLongitude;
			other.altitude = previousUpdatesMap[id].currentAltitude;
			current.latitude = latestUpdatesMap[id].currentLatitude;
			current.longitude = latestUpdatesMap[id].currentLongitude;
			current.altitude = latestUpdatesMap[id].currentAltitude;
			
			//add the distance traveled
			double d = distanceBetween(current, other);
			distanceTraveled[id] = distanceTraveled[id] + d;
			totalDistTraveled = totalDistTraveled + d;
			
			//empty any items from the queue we've reached
			while(!waypointQueues[id].empty() && distanceBetween(waypointQueues[id].front(), current) < COLLISION_THRESHOLD)
			{
				//the front item is reached, pop it and increase our waypoints reached
				struct AU_UAV_ROS::waypoint temp = waypointQueues[id].front();
				waypointQueues[id].pop();
			
				if(waypointQueues[id].empty())
				{
					//we ran out of points x_x
				}
				else
				{
					//add this to our minimum distance
					d = distanceBetween(temp, waypointQueues[id].front());
					minimumTravelDistance[id] = minimumTravelDistance[id] + d;
					totalMinDist = totalMinDist + d;
				}
				
				//modify scoring values
				waypointsAchieved[id]++;
				waypointsTotal++;
				score += WAYPOINT_SCORE;
			}
			
			//TODO: make this have more than 2 planes capable of colliding?
			//time to check for collisions with any other UAVs
			for(int otherID = 0; otherID < id; otherID++)
			{
				other.latitude = latestUpdatesMap[otherID].currentLatitude;
				other.longitude = latestUpdatesMap[otherID].currentLongitude;
				other.altitude = latestUpdatesMap[otherID].currentAltitude;
				
				//check for a conflict
				double d = distanceBetween(current, other);
				if(d < CONFLICT_THRESHOLD)
				{
					//we detected a conflict, increase counter and subtract some points
					numConflicts++;
					score = score + CONFLICT_SCORE;
				}
				
				//check for collisions
				if(d < COLLISION_THRESHOLD)
				{
					//fire & death awaits these two planes...
					planesToDelete.push(id);
					planesToDelete.push(otherID);
					
					//increment our collision counter
					numCollisions++;
					
					//no score penalty, losing a couple planes will be bad enough
				}
			}
		}
		
		//we've parsed everything, delete some planes and display a new output to screen
		while(!planesToDelete.empty())
		{
			int id = planesToDelete.front();
			planesToDelete.pop();
			
			//we're already dead, nothing to do about it
			if(isDead[id]) continue;
			else
			{
				//delete the simulated plane
				//construct the service request
				AU_UAV_ROS::DeleteSimulatedPlane srv;
				srv.request.planeID = id;
				
				//send the request
				printf("\nRequesting to delete plane...\n");
				if(deleteSimulatedPlaneClient.call(srv))
				{
					//printf("Plane with ID #%d has been deleted!\n", planeID);
				}
				else
				{
					ROS_ERROR("Did not receive a response from simulator");
				}
				
				//mark us dead
				timeOfDeath[id] = ros::Time::now() - startTime;
				isDead[id] = true;
				deadPlaneCount++;
			}
		}
		
		displayOutput();
	}
}

int main(int argc, char **argv)
{
	//Standard ROS startup
	ros::init(argc, argv, "evaluator");
	ros::NodeHandle n;
	
	//set up ROS messages
	ros::Subscriber sub = n.subscribe("telemetry", 1000, telemetryCallback);
	
	//set up ROS services
	AU_UAV_ROS::LoadCourse srv;
	createSimulatedPlaneClient = n.serviceClient<AU_UAV_ROS::CreateSimulatedPlane>("create_simulated_plane");
	deleteSimulatedPlaneClient = n.serviceClient<AU_UAV_ROS::DeleteSimulatedPlane>("delete_simulated_plane");
	loadCourseClient = n.serviceClient<AU_UAV_ROS::LoadCourse>("load_course");
	
	//get the file input
	char filename[256];
	printf("\nEnter the filename of the course to load:");
	scanf("%s", filename);
	
	//create all our UAVs
	if(createCourseUAVs(filename))
	{
		//nothing bad happened
	}
	else
	{
		ROS_ERROR("Error creating UAVs");
		return 1;
	}
	
	srv.request.filename = filename;
	
	//countdown to success
	printf("\n");
	printf("Last Plane ID: %d\n", lastPlaneID);
	printf("Loading course into coordinator in 3...\n");
	ros::Duration(1.0).sleep();
	printf("2...\n");
	ros::Duration(1.0).sleep();
	printf("1...\n");
	ros::Duration(1.0).sleep();
	
	//call the load course function on the coordinator
	if(loadCourseClient.call(srv))
	{
		printf("Course loaded successfully!\n");
	}
	else
	{
		ROS_ERROR("Error loading course");
	}
	
	//set the start time to now
	startTime = ros::Time::now();
	
	//planes are created, lets spin for telemetry updates
	ros::spin();
}
