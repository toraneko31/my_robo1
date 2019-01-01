#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>


int main(int argc,char **argv){

  ros::init(argc,argv,"arm_plan_defo");
  ros::NodeHandle nh;
  
  ros::AsyncSpinner spinner(2);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface arm("arm");
  arm.setPoseReferenceFrame("world");
  //default name position
  arm.setNamedTarget("defo");
  arm.move();

  ros::shutdown();
  
  return 0;
}
