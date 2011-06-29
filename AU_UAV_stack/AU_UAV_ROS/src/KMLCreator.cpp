/*
KMLCreator
This component will be used to create standard KML files to be loaded into Google Earth or Google Maps.
This will monitor updates from the planes and then eventually write them to a file.
*/

//standard headers
#include <stdio.h>
#include <map>
#include <queue>

//ros headers
#include "ros/ros.h"
#include "ros/package.h"
#include "AU_UAV_ROS/standardDefs.h"
#include "AU_UAV_ROS/TelemetryUpdate.h"
#include "AU_UAV_ROS/SaveFlightData.h"

#define MAX_LINE_TYPES 6

//at startup we are monitoring the UAVs, maybe change this later
bool isMonitoringTelemetry = true;

//this map links a plane ID to a queue of received points
std::map<int, std::queue<struct AU_UAV_ROS::waypoint> > mapOfPaths;

std::string colorComboArray[] = {"7f0000ff", "7f00ff00", "7fff0000", "7f00ffff", "7fff00ff", "7fffff00"};

/*
telemetryCallback
This is called whenever a new telemetry message is received.  It should simply store the waypoint value
received for writing to a file later.
*/
void telemetryCallback(const AU_UAV_ROS::TelemetryUpdate::ConstPtr& msg)
{
	if(isMonitoringTelemetry)
	{
		//construct waypoint to save
		struct AU_UAV_ROS::waypoint temp;
		temp.latitude = msg->currentLatitude;
		temp.longitude =msg->currentLongitude;
		temp.altitude = msg->currentAltitude;
		
		//save that bad boy
		mapOfPaths[msg->planeID].push(temp);
	}
	else
	{
		//we stopped monitoring which means the data is saved, clean exit time
		exit(0);
	}
}

/*
saveFlightData
This is a service called when it's time to save the file.
*/
bool saveFlightData(AU_UAV_ROS::SaveFlightData::Request &req, AU_UAV_ROS::SaveFlightData::Response &res)
{
	isMonitoringTelemetry = false;
	ROS_INFO("Saving to file %s...", req.filename.c_str());
	
	FILE *fp = fopen((ros::package::getPath("AU_UAV_ROS")+"/flightData/"+req.filename).c_str(), "w");
	if(fp != NULL)
	{
		//opening stuff, never changes
		fprintf(fp, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
		fprintf(fp, "<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n");
		fprintf(fp, "  <Document>\n");
		fprintf(fp, "    <name>Test</name>\n");
		fprintf(fp, "    <open>1</open>\n");
		
		for(int count = 0; count < MAX_LINE_TYPES; count++)
		{
			fprintf(fp, "    <Style id=\"lineType%d\">\n", count);
			fprintf(fp, "      <LineStyle>\n");
			fprintf(fp, "        <color>%s</color>\n", colorComboArray[count % MAX_LINE_TYPES].c_str());
			fprintf(fp, "        <width>4</width>\n");
			fprintf(fp, "      </LineStyle>\n");
			fprintf(fp, "      <PolyStyle>\n");
			fprintf(fp, "        <color>7fffffff</color>\n");
			fprintf(fp, "      </PolyStyle>\n");
			fprintf(fp, "    </Style>\n");
		}
		fprintf(fp, "    <Folder>\n");
		fprintf(fp, "      <name>Paths</name>\n");
		fprintf(fp, "      <visibility>1</visibility>\n");

		//here goes the magic
		std::map<int, std::queue<struct AU_UAV_ROS::waypoint> >::iterator ii;
		
		int count = 0;
		for(ii = mapOfPaths.begin(); ii != mapOfPaths.end(); ii++)
		{
			//create a folder for each path
			fprintf(fp, "      <Folder>\n");
			fprintf(fp, "        <name>UAV #%d</name>\n", ii->first);
			fprintf(fp, "        <visibility>1</visibility>\n");
			
			//create the starting point pin
			fprintf(fp, "        <Placemark>\n");
			fprintf(fp, "          <name>Start #%d</name>\n", ii->first);
			fprintf(fp, "          <visibility>1</visibility>\n");
			fprintf(fp, "          <LookAt>\n");
			fprintf(fp, "            <longitude>%lf</longitude>\n", ii->second.front().longitude);
			fprintf(fp, "            <latitude>%lf</latitude>\n", ii->second.front().latitude);
			fprintf(fp, "            <altitude>0</altitude>\n");
			fprintf(fp, "            <heading>0</heading>\n");
			fprintf(fp, "            <tilt>0</tilt>\n");
			fprintf(fp, "            <range>4451.842204068102</range>\n");
			fprintf(fp, "          </LookAt>\n");
			fprintf(fp, "          <Point>\n");
			fprintf(fp, "            <coordinates>%lf, %lf</coordinates>\n", ii->second.front().longitude, ii->second.front().latitude);
			fprintf(fp, "          </Point>\n");
			fprintf(fp, "        </Placemark>\n");
			
			//this is the path placemark
			fprintf(fp, "        <Placemark>\n");
			fprintf(fp, "          <name>Telemetry #%d</name>\n", ii->first);
			fprintf(fp, "          <visibility>1</visibility>\n");
			fprintf(fp, "          <LookAt>\n");
			fprintf(fp, "            <longitude>%lf</longitude>\n", ii->second.front().longitude);
			fprintf(fp, "            <latitude>%lf</latitude>\n", ii->second.front().latitude);
			fprintf(fp, "            <altitude>0</altitude>\n");
			fprintf(fp, "            <heading>0</heading>\n");
			fprintf(fp, "            <tilt>0</tilt>\n");
			fprintf(fp, "            <range>4451.842204068102</range>\n");
			fprintf(fp, "          </LookAt>\n");
			fprintf(fp, "          <styleUrl>#lineType%d</styleUrl>\n", count);
			count = (count + 1) % MAX_LINE_TYPES;
			fprintf(fp, "          <LineString>\n");
			fprintf(fp, "            <extrude>1</extrude>\n");
			fprintf(fp, "            <tessallate>1</tessallate>\n");
			fprintf(fp, "            <altitudeMode>absolute</altitudeMode>\n");
			fprintf(fp, "            <coordinates>\n");
			
			//for each coordinate saved, we want to write to the file
			AU_UAV_ROS::waypoint temp;
			while(!(ii->second.empty()))
			{
				temp = ii->second.front();
				fprintf(fp, "              %lf, %lf, %lf\n", temp.longitude, temp.latitude, temp.altitude);
				ii->second.pop();
			}
			fprintf(fp, "            </coordinates>\n");
			fprintf(fp, "          </LineString>\n");
			fprintf(fp, "        </Placemark>\n");
			
			//create the ending point pin
			fprintf(fp, "        <Placemark>\n");
			fprintf(fp, "          <name>End #%d</name>\n", ii->first);
			fprintf(fp, "          <visibility>1</visibility>\n");
			fprintf(fp, "          <LookAt>\n");
			fprintf(fp, "            <longitude>%lf</longitude>\n", temp.longitude);
			fprintf(fp, "            <latitude>%lf</latitude>\n", temp.latitude);
			fprintf(fp, "            <altitude>0</altitude>\n");
			fprintf(fp, "            <heading>0</heading>\n");
			fprintf(fp, "            <tilt>0</tilt>\n");
			fprintf(fp, "            <range>4451.842204068102</range>\n");
			fprintf(fp, "          </LookAt>\n");
			fprintf(fp, "          <Point>\n");
			fprintf(fp, "            <coordinates>%lf, %lf</coordinates>\n", temp.longitude, temp.latitude);
			fprintf(fp, "          </Point>\n");
			fprintf(fp, "        </Placemark>\n");
			
			//wrap up this subfolder
			fprintf(fp, "      </Folder>\n");      			
		}

		//closing stuff, never changes
		fprintf(fp, "    </Folder>\n");
		fprintf(fp, "  </Document>\n");
		fprintf(fp, "</kml>\n");
		fclose(fp);
			
		ROS_INFO("Done! It's safe to CTRL-C now.");
		return true;
	}
	else
	{
		ROS_ERROR("Error opening the file");
		res.error = "Error opening the file";
		return false;
	}
}

int main(int argc, char **argv)
{
	//standard ROS startup
	ros::init(argc, argv, "KMLCreator");
	ros::NodeHandle n;
	
	//subscribe to telemetry outputs and create client for the avoid collision service
	ros::Subscriber sub = n.subscribe("telemetry", 1000, telemetryCallback);
	
	//set up services
	ros::ServiceServer saveFlightDataService = n.advertiseService("save_flight_data", saveFlightData);
	
	//needed for ROS to wait for callbacks
	ros::spin();	

	return 0;
}
