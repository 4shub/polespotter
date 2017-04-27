#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/GetPlan.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Pose.h>

geometry_msgs::Pose tmppose;
geometry_msgs::Pose poi;

bool move(x,y){

  ros::ServiceClient client = nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");


  client.waitForExistence();
	std_msgs::String msg;

	tmppose.position.x = 0;
	tmppose.position.y = 0;
	tmppose.orientation.w = cos(0.0 * M_PI / 180 / 2);

		nav_msgs::GetPlan service;

    		service.request.start.header.frame_id = "map";
    		service.request.start.header.stamp = ros::Time::now();
    		service.request.start.pose.position.x = tmppose.position.x;
    		service.request.start.pose.position.y = tmppose.position.y;
    		//service.request.start.pose.orientation.w = tmppose.orientation.w;    

    		service.request.goal.header.frame_id = "map";
    		service.request.goal.header.stamp = ros::Time::now();
    		service.request.goal.pose.position.x = x;
    		service.request.goal.pose.position.y = y;
    		//service.request.goal.pose.orientation.w = poi.orientation.w;

		tmppose.position.x = poi.position.x;
		tmppose.position.y = poi.position.y;
		tmppose.orientation.w = poi.orientation.w;

    		if(client.call(service)){
        		ROS_INFO_STREAM("Service successfully called");
    		}
    		else {
			ROS_ERROR_STREAM("Error while calling service");
    		}

    		for(int i = 0; i < service.response.plan.poses.size(); i++){
    			ROS_INFO_STREAM("plan point " << i << ":" << service.response.plan.poses[i]);

    		}
		/*if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
				msg.data = "COMPLETE";
			}else {
				msg.data = "FAILED";
				ROS_ERROR_STREAM("Failed to reach poi");
				//ros::shutdown();

			}*/
}

/*geometry_msgs::Pose tmppose;
geometry_msgs::Pose poi;

void poiMessageRecieved(const geometry_msgs::Pose& POI) {
    poi = POI;
}

void serviceActivated() {
    ROS_INFO_STREAM("Service received goal");
}

void serviceDone(const actionlib::SimpleClientGoalState& state,
		 const move_base_msgs::MoveBaseResultConstPtr& result) {
    ROS_INFO_STREAM("Service completed");
    ROS_INFO_STREAM("Final state " << state.toString().c_str());
}

void serviceFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& fb) {
    ROS_INFO_STREAM("Service still running");
    ROS_INFO_STREAM("Current pose (x,y,w) " <<
		    fb->base_position.pose.position.x << "," <<
		    fb->base_position.pose.position.y << "," <<
		    fb->base_position.pose.orientation.w);
}

int main(int argc,char **argv) {

  ros::init(argc,argv,"movement");
  ros::NodeHandle nh;
	
  //ros::Subscriber sub = nh.subscribe("/poi",  1000, &poiMessageRecieved);

  ros::Publisher pub = nh.advertise<std_msgs::String>("/polespotter/arrival",  1000);

  ros::ServiceClient client = nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");


  client.waitForExistence();
	std_msgs::String msg;

	tmppose.position.x = 0;
	tmppose.position.y = 0;
	tmppose.orientation.w = cos(0.0 * M_PI / 180 / 2);
poi.position.x = 8;
	poi.position.y = 5;
	poi.orientation.w = cos(0.0 * M_PI / 180 / 2);

  ros::Rate rate(2);


  while (ros::ok()) {



    ros::spinOnce();

	if(tmppose.position.x == poi.position.x && tmppose.position.y == poi.position.y && tmppose.orientation.w == poi.orientation.w){
		continue;
	}else{

		nav_msgs::GetPlan service;

    		service.request.start.header.frame_id = "map";
    		service.request.start.header.stamp = ros::Time::now();
    		service.request.start.pose.position.x = tmppose.position.x;
    		service.request.start.pose.position.y = tmppose.position.y;
    		service.request.start.pose.orientation.w = tmppose.orientation.w;    

    		service.request.goal.header.frame_id = "map";
    		service.request.goal.header.stamp = ros::Time::now();
    		service.request.goal.pose.position.x = poi.position.x;
    		service.request.goal.pose.position.y = poi.position.y;
    		service.request.goal.pose.orientation.w = poi.orientation.w;

		tmppose.position.x = poi.position.x;
		tmppose.position.y = poi.position.y;
		tmppose.orientation.w = poi.orientation.w;

    		if(client.call(service)){
        		ROS_INFO_STREAM("Service successfully called");
    		}
    		else {
			ROS_ERROR_STREAM("Error while calling service");
    		}

    		for(int i = 0; i < service.response.plan.poses.size(); i++){
    			ROS_INFO_STREAM("plan point " << i << ":" << service.response.plan.poses[i]);

    		}
		/*if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
				msg.data = "COMPLETE";
			}else {
				msg.data = "FAILED";
				ROS_ERROR_STREAM("Failed to reach poi");
				//ros::shutdown();

			}*/
		pub.publish(msg);
	}

	




    rate.sleep();
  }

}

/*int main(int argc,char **argv) {

    ros::init(argc,argv,"navigatepoints");
    ros::NodeHandle nh;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
	ac("move_base",true);
    ROS_INFO_STREAM("Waiting for server to be available...");
    while (!ac.waitForServer()) {
    }
    ROS_INFO_STREAM("done!");

    move_base_msgs::MoveBaseGoal goal1;
    move_base_msgs::MoveBaseGoal goal2;
    move_base_msgs::MoveBaseGoal goal3;
    int point = 0;
    
	ros::Rate rate(1);
	bool shoe = true;
	while(ros::ok()) {
		ros::spinOnce();
		if(shoe){
			switch(point){
				case 0:
					goal1.target_pose.header.frame_id = "map";
	    				goal1.target_pose.header.stamp = ros::Time::now();

	    				goal1.target_pose.pose.position.x = 1;
	   				goal1.target_pose.pose.position.y = 1;
	    				goal1.target_pose.pose.orientation.w = cos(0.0 * M_PI / 180 / 2);
	
	    				ac.sendGoal(goal1,&serviceDone,&serviceActivated,&serviceFeedback);
    					//ac.sendGoal(goal);
					break;
				case 1:
					goal2.target_pose.header.frame_id = "map";
	    				goal2.target_pose.header.stamp = ros::Time::now();

	    				goal2.target_pose.pose.position.x = 1;
	   				goal2.target_pose.pose.position.y = 2;
	    				goal2.target_pose.pose.orientation.w = cos(90.0 * M_PI / 180 / 2);

	    				ac.sendGoal(goal2,&serviceDone,&serviceActivated,&serviceFeedback);
					//ac.sendGoal(goal);
					break;
				case 2:
					goal3.target_pose.header.frame_id = "map";
	    				goal3.target_pose.header.stamp = ros::Time::now();
	
	    				goal3.target_pose.pose.position.x = 0;
	   				goal3.target_pose.pose.position.y = 0;
	    				goal3.target_pose.pose.orientation.w = cos(-90.0 * M_PI / 180 / 2);

	    				ac.sendGoal(goal3,&serviceDone,&serviceActivated,&serviceFeedback);
					//ac.sendGoal(goal);
					break;
			}
			
		}
		shoe = ac.waitForResult(ros::Duration(5.0));
		if(shoe){
			if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
				point++;
			}else {
				ROS_ERROR_STREAM("Failed to reach point " << (point+1));
				ros::shutdown();

			}
		}
		//ROS_INFO_STREAM("point is: " << point << " and shoe is: " << shoe);
		rate.reset();
		rate.sleep();
		if(point > 2){
			break;
		}
    }
	
    return 0;
}*/


