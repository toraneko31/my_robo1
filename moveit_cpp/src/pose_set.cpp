#include <ros/ros.h>
#include <iostream>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"

float next_pose[4];
int flag = 0;

void moveCb(const std_msgs::Float32MultiArray::ConstPtr& msg){
    int i=0;
    //std::cout << "pose :";
  for(std::vector<float>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it){
    next_pose[i] = *it;
    //std::cout << next_pose[i] << ",";
    i++;
  }
  //std::cout << std::endl;
}

int main(int argc,char **argv){

  ros::init(argc,argv,"pose_set");
  ros::NodeHandle nh;
  ros::Subscriber get_next;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  get_next = nh.subscribe("/next_pose",10,&moveCb);

  moveit::planning_interface::MoveGroupInterface arm("arm");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  arm.setPoseReferenceFrame("world");
  //default name position
  //arm.setNamedTarget("defo");
  //arm.move();

  ROS_INFO("Reference frame: %s", arm.getPlanningFrame().c_str());
  ROS_INFO("Reference frame: %s", arm.getEndEffectorLink().c_str());
  
    geometry_msgs::PoseStamped start = arm.getCurrentPose();
  
  std::cout << "start_pose :" << start << std::endl;

  ROS_INFO("current pose");
  
  //set position
  ROS_INFO("Moving to prepare pose");
  geometry_msgs::PoseStamped tage;
  tage.header.frame_id = "world";
  if(flag == 0){
    //float point[3] = {0.846,-0.086,-0.32};
    float point[3] = {next_pose[0],next_pose[1],next_pose[2]};
    double set_p[3] = {point[0]*0.5,point[1]*0.5,point[2]*0.5};
    std::cout << "[" << point[0] << "," << point[1] << "," << point[2] << "]" << std::endl;
    std::cout << "[" << set_p[0] << "," << set_p[1] << "," << set_p[2] << "]" << std::endl;
    tage.pose.position.x = start.pose.position.x + set_p[0];
    tage.pose.position.y = start.pose.position.y + set_p[1];
    tage.pose.position.z = start.pose.position.z + set_p[2];
    tage.pose.orientation.w = start.pose.orientation.w;
    flag = 1;
  }
  //tage.pose.orientation.w = 1.0;

  arm.setGoalTolerance(0.3);

  arm.setPoseTarget(tage);

  if(!arm.move()){
    ROS_WARN("Could not move to prepare pose");
    return -1;
  }
  
  ros::shutdown();
  
  return 0;
}
