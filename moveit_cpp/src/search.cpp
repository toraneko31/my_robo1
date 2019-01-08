#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc,char **argv){


  int loop_count = 0;
  int total = 0;

  ros::init(argc,argv,"search");
  ros::NodeHandle nh;
  ros::Rate rate(1); 
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
  int state = 1;
  arm.setNamedTarget("searchR");
  arm.asyncMove();

  //set pose loop "searchR-searchL"
  while(true){
    rate.sleep();
    loop_count++;
    std::cout << loop_count << std::endl;
    if(loop_count > 4){
      if(state == 1){
        arm.setNamedTarget("searchL");
        arm.asyncMove();
        state = 2;
      }
      else if(state == 2){
        arm.setNamedTarget("searchR");
        arm.asyncMove();
        state = 1;
      }
      loop_count = 0;
      total++;
    }

    if(total > 3){
      arm.stop();
      break;
    }


    /*
    arm.setNamedTarget("searchL");
    arm.plan(my_plan);
    arm.execute(my_plan);
    */

  }

  ROS_INFO("Moving End");
  ros::shutdown();
  
  return 0;
}
