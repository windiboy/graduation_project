#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include "geometry_msgs/Pose.h"

class ControlCenter{
public:
    ControlCenter(){
      obj_sub = node_handle.subscribe("/tag_detections/pose",10,&ControlCenter::objectCallback,this);
    }
    void objectCallback(const geometry_msgs::Pose::ConstPtr& msg){
      target = *msg;
      // std::cout << "[objectCallback] target :" << target << std::endl;
    }
    geometry_msgs::Pose target;
private:
    ros::Subscriber obj_sub;
    ros::NodeHandle node_handle;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arm_control");
  ControlCenter center;

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Rate loop_rate(1);

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
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to go to home");
    // std::vector<double> joint_home_positions(6, 0.0);
    // move_group.setJointValueTarget(joint_home_positions);
    // ROS_INFO("Go to home");
    // move_group.move();

    // Start the demo
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    // Planning to a Pose goal
    // ^^^^^^^^^^^^^^^^^^^^^^^
    geometry_msgs::Pose target_pose;
    target_pose = center.target;
    // target_pose.orientation.x = 0.0;
    // target_pose.orientation.y = 0.0;
    // target_pose.orientation.z = 0.0;
    // target_pose.orientation.w = 1;
    move_group.setPoseTarget(target_pose);
    std::cout << "[controller] set pose target :" << target_pose << std::endl;
    // joint space
    // moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    // std::vector<double> joint_group_positions;
    // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    // joint_group_positions[4] = 1.0;  // radians
    // move_group.setJointValueTarget(joint_group_positions);

    // plan and excute
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = false;
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    if (success)
      move_group.execute(my_plan.trajectory_);
    loop_rate.sleep();
  }
  ros::shutdown();
  return 0;
}
