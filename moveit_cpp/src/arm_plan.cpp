#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include <unistd.h>
#include <tf/transform_listener.h>

int camera_no = 0;
int object_centor1[2];
int object_centor2[2];
int object_flag = 0;
int search_flag = 0;
float next_pose[4];
float get_point[4];
float offset[3] = {0,0,0};
int offset_flg = 0;
int move_flg = 1;
int flg = 0;
int fa[3];
int of_count = 0;
int m_count = 0;

void cirCb(const std_msgs::Int32MultiArray::ConstPtr& msg){
  int i=0;
  camera_no = 0;
  for(std::vector<int>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it){
    if(i == 0){
      camera_no = *it;
    }else{
      if(camera_no == 1)object_centor1[i-1] = *it;
      else if(camera_no == 2)object_centor2[i-1] = *it;
    }
    i++;
  }  
}

void moveCb(const std_msgs::Float32MultiArray::ConstPtr& msg){
  int i=0;
  for(std::vector<float>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it){
    get_point[i] = *it;
    i++;
  }
}

int main(int argc,char **argv){

  ros::init(argc,argv,"arm_plan");
  ros::NodeHandle nh;
  ros::Rate rate(100);
  tf::TransformListener ln;

    ros::Subscriber get_next;
  ros::Subscriber get_circle1;
  ros::Subscriber get_circle2;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  get_next = nh.subscribe("/next_pose",10,&moveCb);
  get_circle1 = nh.subscribe("/contour_point1",10,&cirCb);
  get_circle2 = nh.subscribe("/contour_point2",10,&cirCb);

  moveit::planning_interface::MoveGroupInterface arm("arm");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  arm.setPoseReferenceFrame("world");
 int loop_count = 0;
  int step = 0;
  int state = 1;

  arm.setNamedTarget("defo");
  //arm.move();
  arm.plan(my_plan);
  arm.execute(my_plan);
  arm.setNamedTarget("searchR");
  //arm.move();
  arm.plan(my_plan);
  arm.asyncExecute(my_plan);

  search_flag = 1;
  geometry_msgs::PoseStamped tage;

  //default name position
  while(move_flg == 1){
    loop_count++;
    if(loop_count > 100){
       step++;
       std::cout << step << std::endl;
       loop_count = 0;
    }
    if(step > 5){
      if(state == 1){
        arm.setNamedTarget("searchL");
        arm.plan(my_plan);
        arm.asyncExecute(my_plan);

        //arm.asyncMove();
        state = 2;
      }
      else if(state == 2){
        arm.setNamedTarget("searchR");
        arm.plan(my_plan);
        arm.asyncExecute(my_plan);
        //arm.asyncMove();
        state = 1;
      }
      step = 0;
    }
    
    if(search_flag == 1){
      if(object_centor1[0] != 0 && object_centor2[0] != 0){
        if((object_centor1[0] < 600 && object_centor1[1] < 600) && (object_centor1[0] > 200 && object_centor1[1] > 200)){
          if((object_centor2[0] < 600 && object_centor2[1] < 600) && (object_centor2[0] > 200 && object_centor2[1] > 200))
          {
            ROS_INFO("object discovered");
            arm.stop();
            search_flag = 0;
            object_flag = 1;
          }
          std::cout << "level:2" << std::endl;
        }
        std::cout << "level:1" << std::endl;
      }
    }

    rate.sleep();

    geometry_msgs::PoseStamped child;
    geometry_msgs::PoseStamped trans;
    geometry_msgs::PoseStamped link5;

    child.header.frame_id = "link5";
    child.header.stamp = ros::Time(0);
    child.pose.orientation.w = 1.0;
    std::string tar_frame = "world";


    if(object_flag == 1){
      sleep(8);
      tage.header.frame_id = "world";
      next_pose[0] = get_point[0];
      next_pose[1] = get_point[1];
      next_pose[2] = get_point[2];
      while(true){
        arm.setPlanningTime(10.0);
        arm.setPlannerId("RRTConnectkConfigDefault");

        geometry_msgs::PoseStamped start = arm.getCurrentPose();
    
       try{
          ln.waitForTransform(child.header.frame_id,tar_frame,ros::Time(0), ros::Duration(1.0));
          ln.transformPose(tar_frame,child,link5);

          child.pose.position.x = child.pose.position.x + next_pose[0]*0.1;
          child.pose.position.y = child.pose.position.y + next_pose[1]*0.1;
          child.pose.position.z = child.pose.position.z + next_pose[2]*0.1;
          //child.pose.position.x = next_pose[0]*0.4+offset[0];
          //child.pose.position.y = next_pose[1]*0.4+offset[1];
          //child.pose.position.z = next_pose[2]*0.4+offset[2];
          //child.pose.orientation = start.pose.orientation;

          ln.transformPose(tar_frame,child,trans);
          std::cout << "[" <<next_pose[0] << "," << next_pose[1] << "," << next_pose[2] << "]" << std::endl;
          ROS_INFO("x:%f, y:%f,z:%f",link5.pose.position.x,link5.pose.position.y,link5.pose.position.z);
 
          ROS_INFO("x:%f, y:%f,z:%f",trans.pose.position.x,trans.pose.position.y,trans.pose.position.z);
        }
        catch(...){
          ROS_WARN("tf_error");
        }


        arm.setGoalTolerance(0.02);
        //arm.setPoseTarget(tage);
        arm.setPoseTarget(trans);
        if(!arm.plan(my_plan)){
          ROS_WARN("Could not move to prepare pose");
          /*if(fa[0] == 1){
            offset[0]+=0.01;
            of_count++;
          }if(fa[1] == 1){
            offset[0]-=0.01;
            of_count++;
          }
          if(of_count < 5){
            fa[0] = 1;
          }else if(of_count == 5)offset[0] =0.0;
          else if(of_count > 5 && of_count < 10){
            fa[0] = 0;
            fa[1] = 1;
          }else if(of_count >= 10){
            object_flag = 1;
            search_flag = 1;
            break;
          }*/
          m_count++;
          if(m_count>=5){
            move_flg = 0;
            object_flag = 0;
            break;
          }
        }else{
          ROS_INFO("move arm");
          if(!arm.execute(my_plan)){
            ROS_INFO("plan found but not move");
            continue;
          }
          move_flg = 0;
          object_flag = 0;
          m_count++;
          if(m_count >= 5)break;
         }
      }
    }

 } 
  ros::shutdown();

  return 0;
}
