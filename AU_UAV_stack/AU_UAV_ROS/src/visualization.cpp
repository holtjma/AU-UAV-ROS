/*
visualization
This is where telemetry data will be used to create a visual display of what is going on.  Note that both
simulated and real planes will be displayed (can't tell a difference) since they both send telemetry messages.

TODO: nothing is in this file yet, decide on visual system and implement
*/

//standard headers
#include <stdio.h>

//ros headers
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "AU_UAV_ROS/TelemetryUpdate.h"

//set up a publisher to the visualization messages
ros::Publisher markerPub;

/*
telemetryCallback(...)
whenever we receive a telemetry update, we resend some data to rviz
*/
void telemetryCallback(const AU_UAV_ROS::TelemetryUpdate::ConstPtr& msg)
{
	visualization_msgs::Marker points, line_strip;
	points.header.frame_id = line_strip.header.frame_id = "/my_frame";
	points.header.stamp = line_strip.header.stamp = msg->telemetryHeader.stamp;
	points.ns = line_strip.ns = "points_and_lines";
	points.action = line_strip.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
	
	points.id = 0;
	line_strip.id = 1;
	
	points.type = visualization_msgs::Marker::POINTS;
	line_strip.type = visualization_msgs::Marker::LINE_STRIP;
	
	points.scale.x = .2;
	points.scale.y = .2;
	
	line_strip.scale.x = .1;
	line_strip.scale.y = .1;
	
	points.color.g = 1.0;
	points.color.a = 1.0;
	
	line_strip.color.b = 1.0;
	line_strip.color.a = 1.0;
	
	geometry_msgs::Point p;
	p.x = msg->currentLongitude;
	p.y = msg->currentLatitude;
	p.z = msg->currentAltitude;
	
	points.points.push_back(p);
	line_strip.points.push_back(p);
	
	markerPub.publish(points);
	markerPub.publish(line_strip);
}

int main(int argc, char* argv[])
{
	//standard ROS startup
	ros::init(argc, argv, "visualization");
	ros::NodeHandle n;
	
	//subscribe to telemetry updates so we can forward them to rviz
	ros::Subscriber sub = n.subscribe("telemetry", 1000, telemetryCallback);
	
	//set up our publishing
	markerPub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	
	//spin and wait
	ros::spin();
	
	return 0;
}
