#include <ros/ros.h>

int main(int argc,char **argv) {

  ros::init(argc,argv,"main");
  ros::NodeHandle nh;


  ros::Rate rate(2);


  while (ros::ok()) {

    rate.sleep();
  }

}
