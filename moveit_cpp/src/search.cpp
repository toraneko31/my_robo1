#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>


int main(int argc,char **argv){

  int loop_count = 0;

  ros::init(argc,argv,"search");
  ros::NodeHandle nh;
  
  ros::AsyncSpinner spinner(2);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface arm("arm");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  arm.setPoseReferenceFrame("world");
  //default name position
  ROS_INFO("Reference frame: %s", arm.getPlanningFrame().c_str());
  ROS_INFO("Reference frame: %s", arm.getEndEffectorLink().c_str());

  arm.setGoalTolerance(0.1);
  
  //set pose "defo"
  arm.setNamedTarget("defo");
  arm.move();

  ROS_INFO("Moving to prepare pose");

  //set pose "search0"
  arm.setNamedTarget("search0");
  arm.move();
  
  //set pose loop "searchR-searchL"
  while(true){
    arm.setNamedTarget("searchR");
    arm.move();
    arm.setNamedTarget("searchL");
    arm.move();
    loop_count++;

    if(loop_count > 10){
      break;
    }
  }

  ROS_INFO("Moving End");
  ros::shutdown();
  
  return 0;
}
