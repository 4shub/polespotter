#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <vector>

void sendNextLocation(){
    geometry_msgs::Pose location;

}


int main(int argc,char **argv){

  ros::init(argc,argv,"main");
  ros::NodeHandle nh;



  ros::spin();

}