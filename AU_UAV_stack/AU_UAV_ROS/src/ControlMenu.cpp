/*
Controls Menu
This will be the primary UI for now just to control the simulator and coordinator.
*/

//standard headers
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

//ROS headers
#include "ros/ros.h"
#include "AU_UAV_ROS/CreateSimulatedPlane.h"
#include "AU_UAV_ROS/DeleteSimulatedPlane.h"
#include "AU_UAV_ROS/GoToWaypoint.h"
#include "AU_UAV_ROS/LoadPath.h"

//services to the simulator
ros::ServiceClient createSimulatedPlaneClient;
ros::ServiceClient deleteSimulatedPlaneClient;

//services to the coordinator
ros::ServiceClient goToWaypointClient;
ros::ServiceClient loadPathClient;

/*
simulatorMenu()
This is menu for simulator related options.  Specifically, it will be use for UAV creation and deletion.
*/
void simulatorMenu()
{
	//simple menu setup
	int choice = 0;
	
	//loop inside this menu until we go back
	while(choice != 3)
	{
		choice = 0;
		printf("\nSimulator Menu:\n");
		printf("1-Add plane\n");
		printf("2-Delete plane\n");
		printf("3-Back\n");
		printf("Choice:");
		scanf("%d", &choice);
		system("clear");
		
		switch(choice)
		{
			//Add a simulated plane
			case 1:
			{
				//values we need to get started
				double latitude, longitude, altitude;
				double bearing;
				
				//Get user input data
				printf("\nEnter starting latitude, longitude, and altitude (format \"1 2 3\"):");
				scanf("%lf %lf %lf", &latitude, &longitude, &altitude);
				printf("\nEnter starting bearing:");
				scanf("%lf", &bearing);
				
				//construct the service request
				AU_UAV_ROS::CreateSimulatedPlane srv;
				srv.request.startingLatitude = latitude;
				srv.request.startingLongitude = longitude;
				srv.request.startingAltitude = altitude;
				srv.request.startingBearing = bearing;
				
				//send the service request
				printf("\nRequesting to create new plane...\n");
				if(createSimulatedPlaneClient.call(srv))
				{
					printf("New plane with ID #%d has been created!\n", srv.response.planeID);
				}
				else
				{
					ROS_ERROR("Did not receive a response from simulator");
				}
				
				break;
			}
			
			//Delete a simulated plane
			case 2:
			{
				//only value we need
				int planeID;
				
				//get user input
				printf("\nEnter the plane ID to delete:");
				scanf("%d", &planeID);
				
				//construct the service request
				AU_UAV_ROS::DeleteSimulatedPlane srv;
				srv.request.planeID = planeID;
				
				//send the request
				printf("\nRequesting to delete plane...\n");
				if(deleteSimulatedPlaneClient.call(srv))
				{
					printf("Plane with ID #%d has been deleted!\n", planeID);
				}
				else
				{
					ROS_ERROR("Did not receive a response from simulator");
				}
				
				break;
			}
			
			case 3:
			{
				//nothing to do but leave
				break;
			}
			
			default:
			{
				printf("Invalid choice.\n");
				break;
			}
		}
	}
}
/*
pathMenu()
This menu is used for when the user wishes to perform some actions on the path planning aspect of the
coordinator.  This includes simply sending a UAV to a waypoint and loading paths or courses for the UAV.
*/
void pathMenu()
{
	//simple menu setup
	int choice = 0;
	
	//loop until the user asks to go back
	while(choice != 3)
	{
		printf("\nPath Planning Menu:\n");
		printf("1-Go to waypoint\n");
		printf("2-Load path\n");
		printf("3-Back\n");
		printf("Choice:");
		scanf("%d", &choice);
		system("clear");
		
		switch(choice)
		{
			//go to a specified waypoint, this will clear the entire queue for the UAV
			case 1:
			{
				//create service
				AU_UAV_ROS::GoToWaypoint srv;
				
				//get user inputs
				printf("\nEnter the plane ID:");
				scanf("%d", &(srv.request.planeID));
				
				printf("\nEnter latitude, longitude, and altitude (format \"1 2 3\"):");
				scanf("%lf %lf %lf", &(srv.request.latitude), &(srv.request.longitude), &(srv.request.altitude));
				
				//we want to wipe the current queue and go straight to that waypoint
				srv.request.isAvoidanceManeuver = false;
				srv.request.isNewQueue = true;
				
				//call the service
				printf("Sending go to waypoint...\n");
				if(goToWaypointClient.call(srv))
				{
					printf("Waypoint sent successfully!\n");
				}
				else
				{
					ROS_ERROR("Did not receive response from coordinator");
				}
				
				break;
			}
			
			//load a path for a plane
			case 2:
			{
				//create service
				AU_UAV_ROS::LoadPath srv;
				
				//get user inputs
				char filename[256];
				printf("\nEnter the filename:");
				scanf("%s", filename);
				
				int planeID;
				printf("\nEnter the plane ID:");
				scanf("%d", &planeID);
				
				srv.request.planeID = planeID;
				srv.request.filename = filename;
				
				if(loadPathClient.call(srv))
				{
					printf("Path loaded successfully!\n");
				}
				else
				{
					ROS_ERROR("Error loading path");
				}
				break;
			}
			
			//we don't have to do anything for the go back case
			case 3:
			{
				break;
			}
			
			//someone failed to read or type
			default:
			{
				printf("Invalid choice.\n");
				break;
			}
		}
	}
}

/*
main(...)
This just sets up all the ROS stuff and serves as the primary top level menu
*/
int main(int argc, char **argv)
{
	//Standard ROS startup
	ros::init(argc, argv, "ControlsMenu");
	ros::NodeHandle n;
	
	//setup client services
	createSimulatedPlaneClient = n.serviceClient<AU_UAV_ROS::CreateSimulatedPlane>("create_simulated_plane");
	deleteSimulatedPlaneClient = n.serviceClient<AU_UAV_ROS::DeleteSimulatedPlane>("delete_simulated_plane");
	goToWaypointClient = n.serviceClient<AU_UAV_ROS::GoToWaypoint>("go_to_waypoint");
	loadPathClient = n.serviceClient<AU_UAV_ROS::LoadPath>("load_path");
	
	//set up the menu
	int choice = 0;
	
	//loop until the user asks to quit
	while(choice != 3)
	{
		printf("\nStandard Controller Menu:\n");
		printf("1-Simulator Controls\n");
		printf("2-Path Planning\n");
		printf("3-Exit Program\n");
		printf("Choice:");
		scanf("%d", &choice);
		system("clear");
		
		switch(choice)
		{
			case 1:
			{
				simulatorMenu();
				break;
			}
			case 2:
			{
				pathMenu();
				break;
			}
			case 3:
			{
				//nothing to do but leave
				break;
			}
			default:
			{
				printf("Invalid choice.\n");
				break;
			}
		}
	}
	
	return 0;
}
