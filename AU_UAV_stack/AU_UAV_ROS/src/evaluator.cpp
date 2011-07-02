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
#include "AU_UAV_ROS/SaveFlightData.h"

//USER DEFINED EVALUATION SETTINGS
#define TIME_LIMIT 600 //10 minutes
#define WAYPOINT_SCORE 5 //5 points for each waypoint reached
#define CONFLICT_SCORE -1 //-1 point for each conflict during flight

//services to the simulator
ros::ServiceClient createSimulatedPlaneClient;
ros::ServiceClient deleteSimulatedPlaneClient;

//services to the coordinator
ros::ServiceClient loadCourseClient;

//services to the KMLCreator
ros::ServiceClient saveFlightDataClient;

//keep all our updates listed here
std::map<int, AU_UAV_ROS::TelemetryUpdate> previousUpdatesMap;
std::map<int, AU_UAV_ROS::TelemetryUpdate> latestUpdatesMap;

//the list of waypoints to go at
std::map<int, std::queue<AU_UAV_ROS::waypoint> > waypointQueues;

//when our program starts
ros::Time startTime;
ros::Duration delta;

//data about each plane
std::map<int, bool> isDead;
std::map<int, ros::Duration> timeOfDeath;
std::map<int, double> distanceTraveled;
std::map<int, double> waypointDistTraveled;//this value gets changed only when we reach a new waypoint
std::map<int, double> distSinceLastWP;//this stores the distance traveled since our last waypoint
std::map<int, int> waypointsAchieved;
std::map<int, double> minimumTravelDistance;
std::map<int, double> waypointMinTravelDist;//this is distance from the last waypoint to the current

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
int maxAlivePlane = -1;

//file to store our final score sheet
char scoresheetFilename[256];

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
		printf("%d\t\t%lf\t\t%lf\t\t\t%d\t\t\t", id, waypointDistTraveled[id], minimumTravelDistance[id], waypointsAchieved[id]);
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
	printf("Elapsed time:%lf\n", (ros::Time::now() - startTime).toSec());
	printf("Waypoints reached: %d\n", waypointsTotal);
	printf("Number of conflicts: %d\n", numConflicts);
	printf("Number of collisions: %d\n", numCollisions);
	printf("Dead plane count: %d\n", deadPlaneCount);
	if(totalMinDist != 0) printf("Distance actual/distance minimum: %lf\n", totalDistTraveled/totalMinDist);
	printf("Score: %d\n", score);
}

/*
endEvaluation()
This function is called upon termination of the simulation
*/
void endEvaluation()
{
	//open our scoring file
	FILE *fp;
	fp = fopen((ros::package::getPath("AU_UAV_ROS")+"/scores/"+scoresheetFilename+".score").c_str(), "w");
	
	//make sure we got a good open
	if(fp == NULL)
	{
		printf("\nERROR SAVING DATA, COPY TERMINAL OUTPUT!!!\n");
	}
	else
	{
		//dump our data into
		fprintf(fp, "Plane ID\tDistance Traveled(m)\tMinimum Travel(m)\t\tWaypoints Achieved\tTime of Death(s)\n");
		fprintf(fp, "--------\t--------------------\t-----------------\t\t------------------\t----------------\n");
		for(int id = 0; id <= lastPlaneID; id++)
		{
			fprintf(fp, "%d\t\t%lf\t\t%lf\t\t\t%d\t\t\t", id, waypointDistTraveled[id], minimumTravelDistance[id], waypointsAchieved[id]);
			if(isDead[id])
			{
				fprintf(fp, "%lf\n", timeOfDeath[id].toSec());
			}
			else
			{
				fprintf(fp, "ALIVE\n");
			}
		}
		fprintf(fp, "--------\t--------------------\t-----------------\t\t------------------\t----------------\n");
		ros::Duration averageDeath = ros::Duration(0);
		for(int x = 0; x <=lastPlaneID; x++)
		{
			if(isDead[x])
			{
				//we died
				averageDeath+=timeOfDeath[x];
			}
			else
			{
				//we lasted the whole time
				averageDeath+=ros::Duration(TIME_LIMIT);
			}
		}
		fprintf(fp, "Averages:\t%lf\t\t%lf\t\t\t%lf\t\t%lf\n", totalDistTraveled/(lastPlaneID+1), totalMinDist/(lastPlaneID+1), waypointsTotal/(lastPlaneID+1.0), averageDeath.toSec()/(lastPlaneID+1.0));
		fprintf(fp, "\n");
		fprintf(fp, "Totals:\n");
		fprintf(fp, "Elapsed time:%lf\n", (ros::Time::now() - startTime).toSec());
		fprintf(fp, "Waypoints reached: %d\n", waypointsTotal);
		fprintf(fp, "Number of conflicts: %d\n", numConflicts);
		fprintf(fp, "Number of collisions: %d\n", numCollisions);
		fprintf(fp, "Dead plane count: %d of %d\n", deadPlaneCount, (lastPlaneID+1));
		if(totalMinDist != 0) fprintf(fp, "Distance actual/distance minimum: %lf\n", totalDistTraveled/totalMinDist);
		fprintf(fp, "Final Score: %d\n", score);
	
		//close the file
		fclose(fp);
	}
	
	//try to tell the KML Creator to stop
	AU_UAV_ROS::SaveFlightData srv;
	srv.request.filename = (std::string(scoresheetFilename)+".kml");
	if(saveFlightDataClient.call(srv))
	{
		printf("Flight data saved successfully!\n");
	}
	else
	{
		ROS_ERROR("Error saving flight data");
	}
	
	//terminate the program
	exit(0);
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
					maxAlivePlane = planeID;
					
					//set up some base values for a new plane
					latestUpdatesMap[planeID] = AU_UAV_ROS::TelemetryUpdate();
					latestUpdatesMap[planeID].currentLatitude = temp.latitude;
					latestUpdatesMap[planeID].currentLongitude = temp.longitude;
					latestUpdatesMap[planeID].currentAltitude = temp.altitude;
					
					previousUpdatesMap[planeID] = AU_UAV_ROS::TelemetryUpdate();
					previousUpdatesMap[planeID].currentLatitude = temp.latitude;
					previousUpdatesMap[planeID].currentLongitude = temp.longitude;
					previousUpdatesMap[planeID].currentAltitude = temp.altitude;
					
					distanceTraveled[planeID] = 0;
					waypointDistTraveled[planeID] = 0;
					distSinceLastWP[planeID] = 0;
					waypointsAchieved[planeID] = -1; //we get a "free" waypoint
					minimumTravelDistance[planeID] = 0;
					waypointMinTravelDist[planeID] = 0;
					isDead[planeID] = false;
					
					//subtract points to make up for the "free" waypoint at the start
					score = score - 5;
					waypointsTotal--;
				}
					
				//only clear the queue with the first point
				if(isFirstPoint[planeID]) isFirstPoint[planeID] = false;
			}
		}
		
		//if we make it here everything happened according to plan
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
	if(msg->planeID == maxAlivePlane)
	{
		//get the current time
		delta = ros::Time::now() - startTime;
		
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
			//totalDistTraveled = totalDistTraveled + d;
			distSinceLastWP[id] = distSinceLastWP[id] + d;
			
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
					//add the last waypoint to our minimum distance
					minimumTravelDistance[id] = minimumTravelDistance[id] + waypointMinTravelDist[id];
					totalMinDist = totalMinDist + waypointMinTravelDist[id];
					waypointMinTravelDist[id] = distanceBetween(temp, waypointQueues[id].front());
				}
				
				//modify scoring values
				waypointsAchieved[id]++;
				waypointsTotal++;
				score += WAYPOINT_SCORE;
				
				//set our distance traveled for waypoints
				waypointDistTraveled[id] = waypointDistTraveled[id] + distSinceLastWP[id];
				totalDistTraveled = totalDistTraveled + distSinceLastWP[id];
				distSinceLastWP[id] = 0;
			}
			
			//time to check for collisions with any other UAVs
			for(int otherID = 0; otherID < id; otherID++)
			{
				//we assume the wreckage disapates very quickly... lol
				if(isDead[otherID]) continue;
				
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
				timeOfDeath[id] = delta;
				isDead[id] = true;
				deadPlaneCount++;
			}
			
			//if our max plane ID is a dead plane, we no longer receive updates, so we need 
			//to be waiting on a new max for updating the screen
			while(maxAlivePlane >= 0 && isDead[maxAlivePlane])
			{
				//decrement our valu
				maxAlivePlane--;
			}
			
			//check to make sure not all planes are dead
			if(maxAlivePlane < 0)
			{
				displayOutput();
				endEvaluation();
			}
		}
		
		//dump our info
		displayOutput();
		
		//check for the end of times
		if((delta).toSec() > TIME_LIMIT)
		{
			//our time is up, time to write to files and wrap everything up
			endEvaluation();
		}
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
	saveFlightDataClient = n.serviceClient<AU_UAV_ROS::SaveFlightData>("save_flight_data");
	
	//make sure all other nodes have time to activate
	sleep(1);
	system("clear");
	
	//get the file input
	char filename[256];
	printf("\nEnter the filename of the course to load:");
	scanf("%s", filename);
	
	printf("\nEnter the filename for output:");
	scanf("%s", scoresheetFilename);
	
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
	system("clear");
	printf("\n");
	printf("Last Plane ID: %d\n", lastPlaneID);
	printf("KML File: %s.kml\n", scoresheetFilename);
	printf("Score File: %s.score\n", scoresheetFilename);
	printf("Loading course into coordinator in...\n3...\n");
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
