/*
Controls Menu
This will be the primary UI for now just to control the simulator and coordinator.
*/

#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"
#include "AU_UAV_ROS/CreateSimulatedPlane.h"

ros::ServiceClient createSimulatedPlaneClient;

void simulatorMenu(ros::NodeHandle *n)
{
	int choice = 0;
	
	while(choice != 2)
	{
		choice = 0;
		printf("\nSimulator Menu:\n");
		printf("1-Add plane\n");
		printf("2-Exit\n");
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
				printf("\nEnter starting ground speed and bearing (format \"4 5\"):");
				scanf("%lf %lf", &groundSpeed, &bearing);
				
				AU_UAV_ROS::CreateSimulatedPlane srv;
				srv.request.startingLatitude = latitude;
				srv.request.startingLongitude = longitude;
				srv.request.startingAltitude = altitude;
				srv.request.startingGroundSpeed = groundSpeed;
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

int main(int argc, char **argv)
{
	//Standard ROS startup
	ros::init(argc, argv, "ControlsMenu");
	ros::NodeHandle n;
	
	//setup client services
	createSimulatedPlaneClient = n.serviceClient<AU_UAV_ROS::CreateSimulatedPlane>("create_simulated_plane");
	
	int choice = 0;
	
	while(choice != 2)
	{
		printf("\nStandard Controller Menu:\n");
		printf("1-Simulator Controls\n");
		printf("2-Exit\n");
		printf("Choice:");
		scanf("%d", &choice);
		system("clear");
		
		switch(choice)
		{
			case 1:
			{
				simulatorMenu(&n);
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
	
	return 0;
}
