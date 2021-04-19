#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include "geometry_msgs/Pose.h"
#include "std_msgs/Int32.h"
#include "tf/transform_datatypes.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class ControlCenter{
public:
    ControlCenter(){
      gripper_status.data = 0;
      gripper_pub = node_handle.advertise<std_msgs::Int32>("/gripper_control/int32", 1);
    }
    void openGripper()
    {
      gripper_status.data = 0;
      gripper_pub.publish(gripper_status);
    }
    void closeGripper()
    {
      gripper_status.data = 1;
      gripper_pub.publish(gripper_status);
    }
    void pick(moveit::planning_interface::MoveGroupInterface& move_group)
    {
      std::vector<moveit_msgs::Grasp> grasps;
      grasps.resize(1);

      // Setting grasp pose
      // ++++++++++++++++++++++
      grasps[0].grasp_pose.header.frame_id = "base_link";
      tf2::Quaternion orientation;
      orientation.setRPY(-M_PI / 2, 0, 0);
      grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
      grasps[0].grasp_pose.pose.position.x = 0.323;
      grasps[0].grasp_pose.pose.position.y = 0.017;   
      grasps[0].grasp_pose.pose.position.z = 0.130;      
      // Setting pre-grasp approach
      // ++++++++++++++++++++++++++
      grasps[0].pre_grasp_approach.direction.header.frame_id = "base_link";
      grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
      grasps[0].pre_grasp_approach.min_distance = 0.095;
      grasps[0].pre_grasp_approach.desired_distance = 0.115;

      // Setting post-grasp retreat
      // ++++++++++++++++++++++++++
      /* Defined with respect to frame_id */
      grasps[0].post_grasp_retreat.direction.header.frame_id = "base_link";
      /* Direction is set as positive z axis */
      grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
      grasps[0].post_grasp_retreat.min_distance = 0.1;
      grasps[0].post_grasp_retreat.desired_distance = 0.25;

      // Setting posture of eef before grasp
      // +++++++++++++++++++++++++++++++++++
      openGripper();
      // END_SUB_TUTORIAL

      closeGripper();
      // END_SUB_TUTORIAL

      // Call pick to pick up the object using the grasps given
      move_group.pick("object", grasps);
      // END_SUB_TUTORIAL
    }
    std_msgs::Int32 gripper_status;//0-打开 1-关闭
    ros::Publisher gripper_pub;
private:
    ros::NodeHandle node_handle;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arm_control");
  ControlCenter center;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // init
  static const std::string PLANNING_GROUP = "gluon";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  
  // visual
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("dummy");
  visual_tools.deleteAllMarkers();

  // print info
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));
  
  // work flow
  while(ros::ok()){
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to go to home");
    std::vector<double> joint_home_positions(6, 0.0);
    move_group.setJointValueTarget(joint_home_positions);
    move_group.setMaxVelocityScalingFactor(1);
    ROS_INFO("Go to home");
    move_group.move();

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to pick");
    // Planning to a pick
    // ^^^^^^^^^^^^^^^^^^^^^^^
    center.pick(move_group);
  }
  ros::shutdown();
  return 0;
}
