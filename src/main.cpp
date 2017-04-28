#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/GetPlan.h>
#include <vector>
#include <string>

// foreach
#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

// defaults
const int ZONE_TABLE_MAX_X = 8;
const int ZONE_TABLE_MAX_Y = 8;
const int ZONE_COUNT = ZONE_TABLE_MAX_X*2 * ZONE_TABLE_MAX_Y*2;

// Declare handler


geometry_msgs::Point currentLocation;
std::vector<geometry_msgs::Point> zoneList(ZONE_COUNT);
std::vector<geometry_msgs::Point> poleList;

/*
Zones managed:
   0
(-8,8)(-6,8)(-4,8)(-2,8)(0,8)(2,8)(4,8)(6,8)(8,8)
(-8,6)(-6,6)(-4,6)...       ...(4,-6)(6,-6)(8,-6)
(-8,-8)(-6,-8)(-4,-8)...    ...(4,-8)(6,-8)(8,-8)
                                             63
*/
int zoneHandled = 0; // which zone of operation the robot is currently handling

void setOriginLocation(){
  currentLocation.x = 0;
  currentLocation.y = 0;
  currentLocation.z = 0;
}

void setZoneIndicies(){
  std::vector<geometry_msgs::Point> zones(ZONE_COUNT);

  int x_index = ZONE_TABLE_MAX_X;
  int y_index = ZONE_TABLE_MAX_Y;

  for(int i = 0; i < ZONE_TABLE_MAX_Y; i++){
    for(int j = 0; j < ZONE_TABLE_MAX_X; j++){
      zones[j].x = x_index;
      zones[j].y = y_index;

      x_index = x_index - 2;
    }

    y_index = y_index - 2;
  }

  zoneList = zones;
}

void incrementZone(){
  zoneHandled++;
}

void addPole(geometry_msgs::Point poleLocation){
  poleList.push_back(poleLocation);
  ROS_INFO_STREAM("ADDED POLE AT x:" << poleLocation.x << " y:" << poleLocation.y);
}

void verifyAndInsertPole(geometry_msgs::Point poleLocation){
  for(int i = 0; i < poleList.size(); i++){
    float x_loc = poleList[i].x;
    float y_loc = poleList[i].y;
    float errorMargin = 0.25;
    if(x_loc + errorMargin >= poleLocation.x && 
       x_loc - errorMargin <= poleLocation.x &&
       y_loc + errorMargin >= poleLocation.y &&
       y_loc - errorMargin <= poleLocation.y
      ){
        
        // if we find a conflict
        return;
    }
  }

  addPole(poleLocation);
  return;
}




void moveRobot(ros::NodeHandle nh, float x, float y){
  // initialize subscriber base
  ros::ServiceClient client = nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
  client.waitForExistence();

  nav_msgs::GetPlan service;

  ROS_INFO("init: x = %f, y = %f", currentLocation.x, currentLocation.y);

  service.request.start.header.frame_id = "map";
  service.request.start.header.stamp = ros::Time::now();
  service.request.start.pose.position.x = currentLocation.x;
  service.request.start.pose.position.y = currentLocation.y;
  service.request.start.pose.orientation.w = 0;    

  service.request.goal.header.frame_id = "map";
  service.request.goal.header.stamp = ros::Time::now();
  service.request.goal.pose.position.x = x;
  service.request.goal.pose.position.y = y;
  service.request.goal.pose.orientation.w = 0;

  currentLocation.x = x;
  currentLocation.y = y;

  if(client.call(service)){
    ROS_INFO_STREAM("Service successfully called");

   forEach(const geometry_msgs::PoseStamped &p, service.response.plan.poses) {
      ROS_INFO("x = %f, y = %f", p.pose.position.x, p.pose.position.y);
   }

  } else {
    ROS_ERROR_STREAM("Error while calling service");
  }

 


  /*if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("MOVED SUCCESFULLY");
    //scanPoles();
    incrementZone();
    moveRobot(zoneList[zoneHandled].x, zoneList[zoneHandled].y);
	} else {
    ROS_INFO("MOVED FAILED");
    incrementZone();
    moveRobot(zoneList[zoneHandled].x, zoneList[zoneHandled].y);
	}*/

  ros::spin();
}


void scanPoles(){
  
}



int main(int argc,char **argv){
  ros::init(argc,argv,"main");

  ros::NodeHandle nh;
  
  // initialize routes
  setOriginLocation();
  setZoneIndicies();

  

  moveRobot(nh, zoneList[zoneHandled].x, zoneList[zoneHandled].y);

  

}