#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>
#include <iostream>

int main(int argc,char **argv){
  ros::init(argc,argv,"gripper_plan");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface ee("ee");
  ee.setPoseReferenceFrame("world");
  
  //std::vector<std::string> joints = ee.getJoints();

  //std::copy(begin(joints), end(joints), std::ostream_iterator<std::string>(std::cout, "\n"));


  std::map<std::string, double> joints;
  joints["joint5"] = 0.82;
  joints["joint6"] = 0.82;
  ee.setJointValueTarget(joints);
  if(!ee.move()){
    ROS_WARN("failured gripper move");
    return -1;
  }
 

  ros::shutdown();

  return 0; 
}
