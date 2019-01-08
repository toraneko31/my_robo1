#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>


int main(int argc,char **argv){

  ros::init(argc,argv,"stop_test");
  ros::NodeHandle nh;
  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface arm("arm");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan; 
  arm.setPoseReferenceFrame("world");
  arm.setNamedTarget("defo");
  arm.asyncMove();

  //default name position
  while(true){
    geometry_msgs::PoseStamped cur = arm.getCurrentPose();
    std::cout << cur.pose.position << std::endl;
    if(cur.pose.position.z > 1.7){
      ROS_INFO("stop arm move");
      arm.stop();
      break;
    }
  }
  ros::shutdown();

  return 0;
}
