#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>


int main(int argc,char **argv){

  ros::init(argc,argv,"local_test");
  ros::NodeHandle nh;
  
  ros::AsyncSpinner spinner(2);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface arm("arm");
  moveit::planning_interface::MoveGroupInterface::Plan my;
  arm.setPoseReferenceFrame("world");
  //default name position
  arm.setNamedTarget("defo");
  arm.plan(my);
  arm.execute(my);

  geometry_msgs::PoseStamped cur = arm.getCurrentPose();

  std::cout << cur.pose.position << std::endl;

  geometry_msgs::PoseStamped tage;
  int count = 10;
  float gx = 0.1;
  float gz = -0.2;
  float step_x = gx/(float)count;
  float step_z = gz/(float)count;
  float inter = 0.3/(float)count;

  arm.setGoalTolerance(0.03);

  tage.header.frame_id = "link5";

  for(int i = 0;i < count;i++){
    tage.pose.position.x = step_x;
    tage.pose.position.y = 0;
    tage.pose.position.z = step_z;
    /*tage.header.frame_id = "world";
      tage.pose.position.x = cur.pose.position.x + 0.2;
      tage.pose.position.y = cur.pose.position.y;
      tage.pose.position.z = cur.pose.position.z - 0.5;
     */

    arm.setPoseTarget(tage);
    arm.plan(my);
    if(!arm.execute(my)){
      std::cout << "miss move" << std::endl;
      return -1;
    }
    std::cout << "pass :" << i << std::endl;
  }

  geometry_msgs::PoseStamped end = arm.getCurrentPose();

  std::cout << end.pose.position << std::endl;

  ros::shutdown();
  
  return 0;
}
