/*
generator.cpp
This file is intended to generate massive amounts of test cases with simple configuration

Compiled with g++.
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string>

//*********************************************************************
//USER DEFINED SETTINGS - you can put as many options as you want in {}

//coordinates of top-left corner of field
#define WEST_MOST_LONGITUDE -85.490356
#define NORTH_MOST_LATITUDE 32.606573
#define MIN_ALTITUDE 400
#define MAX_ALTITUDE 400

//number of test cases for each setting
#define COURSES_PER_SETTING 3

//number of points per plane
#define WAYPOINTS_PER_PLANE 50

//amount of space in meters between each plane at start
#define BUFFER_SPACE 36.0 //meters

//output directory
#define OUTPUT_DIRECTORY "/home/matt/generator/courses/quickgen"

//list of the number of planes to run tests on
const int numPlanes[] = {4, 8, 16, 32};

//list of the field size lengths; {500} will create tests for 500m by 500m fields
const int fieldSizes[] = {500, 1000}; //meters

//seed values for RNG
const int seed = time(NULL);

//END USER DEFINED SETTINGS
//*********************************************************************

//1 degree latitude ~= 111.2 km
#define METERS_TO_LATITUDE (1.0/111200.0)
#define EARTH_RADIUS 6371000.0 //meters

#define DEGREES_TO_RADIANS (M_PI/180.0)
#define RADIANS_TO_DEGREES (180.0/M_PI)

struct waypoint
{
	double latitude;
	double longitude;
	double altitude;
};

/*
distanceBetween(...)
Returns the distance in meters between the two waypoints provided.  Note that it does take into account
the earth's curvature.
NOTE: copied from other source code
*/
double distanceBetween(struct waypoint first, struct waypoint second)
{
	//difference in latitudes in radians
	double lat1 = first.latitude*DEGREES_TO_RADIANS;
	double lat2 = second.latitude*DEGREES_TO_RADIANS;
	double long1 = first.longitude*DEGREES_TO_RADIANS;
	double long2 = second.longitude*DEGREES_TO_RADIANS;
	
	double deltaLat = lat2 - lat1;
	double deltaLong = long2 - long1;
	
	//haversine crazy math, should probably be verified further beyond basic testing
	//calculate distance from current position to destination
	double a = pow(sin(deltaLat / 2.0), 2);
	a = a + cos(lat1)*cos(lat2)*pow(sin(deltaLong/2.0), 2);
	a = 2.0 * asin(sqrt(a));
	
	return EARTH_RADIUS * a;
}

void generateCourse(int numberOfPlanes, int fieldLength, int courseNumber)
{
	//let the user know what we're doing
	//printf("Generating course #%d with %d planes and %d field length...\n", courseNumber, numberOfPlanes, fieldLength);
	
	char filename[1000];
	sprintf(filename, "%s/final_%d_%dm_%d.course", OUTPUT_DIRECTORY, numberOfPlanes, fieldLength, courseNumber);
	
	struct waypoint planeStarts[numberOfPlanes];
	
	FILE *fp;
	fp = fopen(filename, "w");
	
	if(fp == NULL)
	{
		printf("ERROR: Couldn't open file %s!\n", filename);
	}
	else
	{
		struct waypoint newPoint;
		int altitudeDifference = MAX_ALTITUDE - MIN_ALTITUDE;
		
		fprintf(fp, "#Auburn University ATTRACT Project - AU_UAV_ROS Sub-project\n");
		fprintf(fp, "#Randomly generated course file\n");
		fprintf(fp, "#Settings:\n");
		fprintf(fp, "#\tNumber of planes: %d\n", numberOfPlanes);
		fprintf(fp, "#\tField size: %d meters by %d meters\n", fieldLength, fieldLength);
		fprintf(fp, "#\tWaypoints per plane: %d\n", WAYPOINTS_PER_PLANE);
		fprintf(fp, "#\tNorth-west corner: (%lf, %lf)\n", NORTH_MOST_LATITUDE, WEST_MOST_LONGITUDE);
		fprintf(fp, "#\tAltitude range: %d - %d\n", MIN_ALTITUDE, MAX_ALTITUDE);
		fprintf(fp, "\n");
		fprintf(fp, "#Starting waypoints\n");
		fprintf(fp, "#Plane ID\tLatitude\tLongitude\tAltitude\n");
		for(int planeid = 0; planeid < numberOfPlanes; planeid++)
		{
			//latitude is easy to calculate
			newPoint.latitude = NORTH_MOST_LATITUDE - METERS_TO_LATITUDE*(rand()%(fieldLength+1));
			
			//longitude requires a little more stuff
			int longitudeMeters = rand()%(fieldLength+1);
			double temp = pow(sin((longitudeMeters/EARTH_RADIUS)/2.0), 2);
			temp = temp / (sin(M_PI/2.0 - newPoint.latitude*DEGREES_TO_RADIANS)*sin((M_PI/2.0)-newPoint.latitude*DEGREES_TO_RADIANS));
			newPoint.longitude = WEST_MOST_LONGITUDE + 2.0*RADIANS_TO_DEGREES*asin(sqrt(temp));
			
			//altitude is easy too
			newPoint.altitude = MIN_ALTITUDE + rand()%(altitudeDifference+1);
			
			//make sure our distance between each other point is enough
			bool isTooClose = false;
			for(int i = 0; i < planeid && !isTooClose; i++)
			{
				if(distanceBetween(newPoint, planeStarts[i]) < BUFFER_SPACE)
				{
					//subtract a value to put us back where we were and re-roll the dice
					isTooClose = true;
					planeid--;
				}
			}
			if(isTooClose) continue;

			//save our point for later
			planeStarts[planeid] = newPoint;
			
			//output to our file
			fprintf(fp, "%d\t\t%lf\t%lf\t%lf\n", planeid, newPoint.latitude, newPoint.longitude, newPoint.altitude);
		}
		fprintf(fp, "\n");
		
		//for each plane, create a list of values
		for(int planeid = 0; planeid < numberOfPlanes; planeid++)
		{
			//create a label
			fprintf(fp, "#Plane ID: %d\n", planeid);
		
			for(int x = 0; x < WAYPOINTS_PER_PLANE; x++)
			{
				//latitude is easy to calculate
				newPoint.latitude = NORTH_MOST_LATITUDE - METERS_TO_LATITUDE*(rand()%(fieldLength+1));
			
				//longitude requires a little more stuff
				int longitudeMeters = rand()%(fieldLength+1);
				double temp = pow(sin((longitudeMeters/EARTH_RADIUS)/2.0), 2);
				temp = temp / (sin(M_PI/2.0 - newPoint.latitude*DEGREES_TO_RADIANS)*sin((M_PI/2.0)-newPoint.latitude*DEGREES_TO_RADIANS));
				newPoint.longitude = WEST_MOST_LONGITUDE + 2.0*RADIANS_TO_DEGREES*asin(sqrt(temp));
			
				//altitude is easy too
				newPoint.altitude = MIN_ALTITUDE + rand()%(altitudeDifference+1);
			
				//output to our file
				fprintf(fp, "%d\t\t%lf\t%lf\t%lf\n", planeid, newPoint.latitude, newPoint.longitude, newPoint.altitude);
			}
			fprintf(fp, "\n");
		}
		
		//close the file
		fclose(fp);
	}
}

int main()
{
	int numPlaneLength = sizeof(numPlanes) / sizeof(int);
	int fieldSizesLength = sizeof(fieldSizes) / sizeof(int);
	
	srand(seed);

	printf("\nSettings:\n");
	printf("Output directory: %s\n", OUTPUT_DIRECTORY);
	printf("Course Corner: (%lf, %lf)\n", NORTH_MOST_LATITUDE, WEST_MOST_LONGITUDE);
	printf("Number of courses per setting: %d\n", COURSES_PER_SETTING);
	printf("Plane buffer space: %lf meters\n", BUFFER_SPACE);
	printf("Number of planes list: {");
	for(int i = 0; i < numPlaneLength; i++)
	{
		printf("%d", numPlanes[i]);
		if(i+1 != numPlaneLength) printf(", ");
		else printf("}\n");
	}
	printf("Field sizes list: {");
	for(int i = 0; i < fieldSizesLength; i++)
	{
		printf("%d", fieldSizes[i]);
		if(i+1 != fieldSizesLength) printf(", ");
		else printf("}\n");
	}
	
	printf("Generating...");
	for(int numPlaneIndex = 0; numPlaneIndex < numPlaneLength; numPlaneIndex++)
	{
		for(int fieldSizesIndex = 0; fieldSizesIndex < fieldSizesLength; fieldSizesIndex++)
		{
			for(int courseNum = 0; courseNum < COURSES_PER_SETTING; courseNum++)
			{
				generateCourse(numPlanes[numPlaneIndex], fieldSizes[fieldSizesIndex], courseNum+1);
			}
			//printf("\n");
		}
	}
	printf("Done!\n\n");
	
	return 0;
}
