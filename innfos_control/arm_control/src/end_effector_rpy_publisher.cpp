#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "tf/transform_datatypes.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"

class RpyPublisher{
private:
    ros::NodeHandle nh_;
public:
    ros::Publisher pub;
    RpyPublisher(){
        pub = nh_.advertise<std_msgs::Float64MultiArray>("/arm_control/end_effector_rpy", 1);
    }
};
int main(int argc, char** argv)
{
  ros::init(argc, argv, "gripper_control");
  RpyPublisher rpy;

  ros::AsyncSpinner spinner(1);
  spinner.start();
  // init
  static const std::string PLANNING_GROUP = "gluon";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  ros::Rate loop_rate(10);
  while (ros::ok()){
    geometry_msgs::PoseStamped pose = move_group.getCurrentPose();
    tf::Quaternion quat;
    tf::quaternionMsgToTF(pose.pose.orientation, quat);
    double roll, pitch, yaw;//定义存储r\p\y的容器
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    std::cout << "Quaternion" << pose.pose << std::endl;
    std::cout << "roll, pitch, yaw: " << roll*360/3.14 <<", "<< pitch*360/3.14 <<", "<< yaw*360/3.14 <<", "<< std::endl;

    std_msgs::Float64MultiArray msg;
    msg.data.push_back(roll);
    msg.data.push_back(pitch);
    msg.data.push_back(yaw);
    rpy.pub.publish(msg);
    loop_rate.sleep();
  }
  ros::shutdown();
  return 0;
}