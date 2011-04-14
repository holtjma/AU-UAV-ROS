/*
Controls Menu
This will be the primary UI for now just to control the simulator and coordinator.
*/

#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"

void simulatorMenu(ros::NodeHandle *n)
{
	int choice = 0;
	
	while(choice != 2)
	{
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
