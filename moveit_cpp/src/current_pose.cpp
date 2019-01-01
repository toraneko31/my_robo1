#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>


int main(int argc,char **argv){

  ros::init(argc,argv,"current_pose");
  ros::NodeHandle nh;
  
  ros::AsyncSpinner spinner(2);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface arm("arm");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  arm.setPoseReferenceFrame("world");
  //default name position
  //arm.setNamedTarget("defo");
  //arm.move();

  ROS_INFO("Reference frame: %s", arm.getPlanningFrame().c_str());
  ROS_INFO("Reference frame: %s", arm.getEndEffectorLink().c_str());


  geometry_msgs::PoseStamped start_pose = arm.getCurrentPose();
  
  std::cout << "start_pose :" << start_pose << std::endl;

  std::vector<std::string> pose_name = arm.getNamedTargets();
  
  for(int i=0;i<4;i++){
    std::cout << "pose_name :" << pose_name[i] << std::endl;
  }
  
  ros::shutdown();
  
  return 0;
}
