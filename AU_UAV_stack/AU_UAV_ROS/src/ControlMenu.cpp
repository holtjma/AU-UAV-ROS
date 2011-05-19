/*
Controls Menu
This will be the primary UI for now just to control the simulator and coordinator.
*/

#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"
#include "AU_UAV_ROS/CreateSimulatedPlane.h"
#include "AU_UAV_ROS/GoToWaypoint.h"

ros::ServiceClient createSimulatedPlaneClient;
ros::ServiceClient goToWaypointClient;

void simulatorMenu()
{
	int choice = 0;
	
	while(choice != 2)
	{
		choice = 0;
		printf("\nSimulator Menu:\n");
		printf("1-Add plane\n");
		printf("2-Back\n");
		printf("Choice:");
		scanf("%d", &choice);
		system("clear");
		
		switch(choice)
		{
			case 1:
			{
				double latitude, longitude, altitude;
				double groundSpeed, bearing;
				printf("\nEnter starting latitude, longitude, and altitude (format \"1 2 3\"):");
				scanf("%lf %lf %lf", &latitude, &longitude, &altitude);
				printf("\nEnter starting bearing (format \"4\"):");
				scanf("%lf", &bearing);
				
				AU_UAV_ROS::CreateSimulatedPlane srv;
				srv.request.startingLatitude = latitude;
				srv.request.startingLongitude = longitude;
				srv.request.startingAltitude = altitude;
				srv.request.startingBearing = bearing;
				
				printf("\nRequesting to create new plane...\n");
				
				if(createSimulatedPlaneClient.call(srv))
				{
					printf("New plane with ID #%d has been created!\n", srv.response.planeID);
				}
				else
				{
					ROS_ERROR("Did not receive response from simulator");
				}
				
				break;
			}
			case 2:
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

void pathMenu()
{
	int choice = 0;
	
	while(choice != 2)
	{
		printf("\nPath Planning Menu:\n");
		printf("1-Go to waypoint\n");
		printf("2-Back\n");
		printf("Choice:");
		scanf("%d", &choice);
		system("clear");
		
		switch(choice)
		{
			case 1:
			{
				AU_UAV_ROS::GoToWaypoint srv;
				
				printf("\nEnter the plane ID:");
				scanf("%d", &(srv.request.planeID));
				
				printf("\nEnter latitude, longitude, and altitude (format \"1 2 3\"):");
				scanf("%lf %lf %lf", &(srv.request.latitude), &(srv.request.longitude), &(srv.request.altitude));
				
				//default to not being avoidance and that this is a go here now point
				srv.request.isAvoidanceManeuver = false;
				srv.request.isNewQueue = true;
				
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
			case 2:
			{
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

int main(int argc, char **argv)
{
	//Standard ROS startup
	ros::init(argc, argv, "ControlsMenu");
	ros::NodeHandle n;
	
	//setup client services
	createSimulatedPlaneClient = n.serviceClient<AU_UAV_ROS::CreateSimulatedPlane>("create_simulated_plane");
	goToWaypointClient = n.serviceClient<AU_UAV_ROS::GoToWaypoint>("go_to_waypoint");
	
	int choice = 0;
	
	while(choice != 3)
	{
		printf("\nStandard Controller Menu:\n");
		printf("1-Simulator Controls\n");
		printf("2-Path Planning\n");
		printf("3-Exit\n");
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
