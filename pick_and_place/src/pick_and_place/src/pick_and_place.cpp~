#include <ros/ros.h>
#include <tf/tf.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <string>
#include <vector>
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
const float PI = 3.14159;
class PickAndPlace
{

private:
  ros::NodeHandle nh_;
  ros::Subscriber pick_and_place_sub_;
  ros::Subscriber height_sub_;
  moveit::planning_interface::MoveGroup arm_;
  float z_up,height;

public:
  PickAndPlace(const std::string name) :
   nh_("~"), arm_("arm")
  {
     arm_.allowReplanning(true);
     arm_.setPoseReferenceFrame("base_link");
     z_up = 0.0;
   
     // Allow some leeway in position (meters) and orientation (radians)这里的参数不定
     arm_.setGoalPositionTolerance(0.005);//m
     arm_.setGoalOrientationTolerance(0.1);//radian
     height_sub_=nh_.subscribe("/height_get", 1, &PickAndPlace::heightcb, this);
    
     pick_and_place_sub_=nh_.subscribe("/pick_and_place", 1, &PickAndPlace::Msgcb, this);
  }
  
  void heightcb(const std_msgs::Float64::ConstPtr& height_)
  {
    ROS_INFO("[pick and place] Got goal from topic! %s", "/height_get");
    height=height_->data+0.01;
    std::cout<<"height = "<<height<<std::endl;
    height_sub_.shutdown();
  }
  void Msgcb(const geometry_msgs::PoseConstPtr& msg)
  {
    ROS_INFO("[pick and place] Got goal from topic! %s", "/pick_and_place");
    pickandplace(*msg);
    pick_and_place_sub_.shutdown();
  }
  void pickandplace(const geometry_msgs::Pose& start_pose)
  {
    ROS_INFO("[pick and place] Picking");
 
    geometry_msgs::Pose target_pose;
    target_pose=start_pose;
    z_up = start_pose.position.z+0.5*height+0.02; 
    target_pose.position.x -= 0.01;
    target_pose.position.y -= 0.02;
    target_pose.position.z = z_up;
    if (moveArmTo(target_pose) == false)
      return;
    ROS_INFO("top grasp done!");
    
    sleep(1.0);
   /* reset */
    geometry_msgs::PoseStamped target_pose_Stamped;
    target_pose_Stamped.header.frame_id = "base_link";
    target_pose_Stamped.pose.orientation.w = 0.631477;
    target_pose_Stamped.pose.orientation.x = -0.0099124;
    target_pose_Stamped.pose.orientation.y = -0.77529;
    target_pose_Stamped.pose.orientation.z = -0.0080712;
    target_pose_Stamped.pose.position.x = 0.080525;
    target_pose_Stamped.pose.position.y = 0.030499;
    target_pose_Stamped.pose.position.z = 0.81721;
    arm_.setPoseTarget(target_pose_Stamped);
    arm_.move();
    ROS_INFO("RESET done!");
  }
  
    bool moveArmTo(const geometry_msgs::Pose& target)
  {
    int attempts = 0;
   // ROS_DEBUG("[pick and place] Move arm to [%.2f, %.2f, %.2f, %.2f]",
             //target.position.x, target.position.y, target.position.z, tf::getYaw(target.orientation));
    
    ROS_INFO("[pick and place] Move arm to [%.2f, %.2f, %.2f]",
             target.position.x, target.position.y, target.position.z);
    while (attempts < 2)
    {
      geometry_msgs::PoseStamped modiff_target;
      modiff_target.header.frame_id = "base_link";
      modiff_target.pose = target;

      float  x = modiff_target.pose.position.x-0.1;
      float  y = modiff_target.pose.position.y-0.03;
      float  z = modiff_target.pose.position.z-0.435;
      float  d = sqrt(x*x + y*y);
      std::cout<<" x = "<<x<<" y = "<<y<<" z = "<<z<<std::endl;
 
      float yaw = acos(x/d);
      if (y<0)
      {
          yaw=-yaw;
      }
      if (d > 0.27)
      {
        // Maximum reachable distance by the turtlebot arm is 30 cm
        ROS_ERROR("Target pose out of reach [%f > %f]", d, 0.27);
    
        return false;
       }
       Eigen::Quaternionf quat_1(Eigen::AngleAxis<float>(PI/2, Eigen::Vector3f(0,1,0)));
       Eigen::Quaternionf quat_2(Eigen::AngleAxis<float>(yaw, Eigen::Vector3f(0,0,1)));
       Eigen::Quaternionf quat_3 = quat_2*quat_1;
       modiff_target.pose.orientation.x = quat_3.x();
       modiff_target.pose.orientation.y = quat_3.y();
       modiff_target.pose.orientation.z = quat_3.z();
       modiff_target.pose.orientation.w = quat_3.w();

      if (arm_.setPoseTarget(modiff_target) == false)
      {
        ROS_ERROR("Set pose target [%.2f, %.2f, %.2f, %.2f] failed",
                  modiff_target.pose.position.x, modiff_target.pose.position.y, modiff_target.pose.position.z,
                  tf::getYaw(modiff_target.pose.orientation));
       
        return false;
      }
      moveit::planning_interface::MoveGroup::Plan my_plan;
      moveit_msgs::MoveItErrorCodes errorCodes = arm_.plan(my_plan);
      ROS_INFO("Planning result: %d", errorCodes.val);
      sleep(1.0);
      moveit::planning_interface::MoveItErrorCode result = arm_.move();//执行结果是否正确
      if (bool(result) == true)
      {
        return true;
      }
      else
      {
        ROS_ERROR("[pick and place] Move to target failed at attempt %d",
                   attempts + 1);
      }
      attempts++;
    }

    ROS_ERROR("[pick and place] Move to target failed after %d attempts", attempts);
    
    return false;
  }
 
};
   
int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_and_place");

  // Setup an asynchronous spinner as the move groups operations need continuous spinning
  ros::AsyncSpinner spinner(2);
  spinner.start();

  PickAndPlace server("pick_and_place");
  ros::spin();
  return 0;
}
