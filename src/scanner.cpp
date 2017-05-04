// ConsoleApplication7.cpp : Defines the entry point for the console application.


#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>


void msgReceived(const sensor_msgs::LaserScan& msg) {
ROS_INFO_STREAM(" [ 1] ");
	/*int count = 0;
	float ai = msg.angle_increment;
	std::vector <float> angle;

	for (float i = ai + msg.angle_min; i < msg.angle_max; i = +ai) {
		angle.push_back(ai);
		count++;
	}
ROS_INFO_STREAM(" [ 2] ");
	std::vector <float> xaxis (count);
	std::vector <float> yaxis (count);
	std::vector <float> intensity;
	//vector <vector <float> > coordinates


ROS_INFO_STREAM(" [ 3] ");
	for (int i = 0; i < count++; i++) {
		float x;
		float y;
		x = cos(angle[i])*msg.ranges[i];
		y = sin(angle[i])*msg.ranges[i];
		intensity.push_back(msg.intensities[i]);
		xaxis.push_back(x);
		yaxis.push_back(y);
	}
	for (int i = 0; i < xaxis.size(); i++) {
		ROS_INFO_STREAM(" [" << xaxis.at(i) << " , " << yaxis.at(i) << "] ");
	}
	
	ROS_INFO_STREAM(" [ 4] ");*/
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "laser_scan_publisher");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/scan", 1000, &msgReceived);
	ROS_INFO_STREAM(" [ 5] ");
	while(ros::ok()){
		ros::spin();
		//ROS_INFO_STREAM("looping");
	}

}
