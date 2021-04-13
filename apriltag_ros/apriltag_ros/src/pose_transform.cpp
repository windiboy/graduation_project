#include "apriltag_ros/continuous_detector.h"
#include "geometry_msgs/Pose.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include <tf/transform_listener.h>

class TransForm{
  private:
    ros::NodeHandle nh_;
    ros::Subscriber sub;
    ros::Publisher pub;
    tf::TransformListener listener;
    geometry_msgs::PoseStamped base_pose;
    geometry_msgs::PoseStamped camera_pose;
  public:
    TransForm() : nh_("~")
    {
      sub = nh_.subscribe("/tag_detections",10,&TransForm::objectCallback,this);
      pub = nh_.advertise<geometry_msgs::Pose>("/tag_detections/pose", 1);
    }
    void objectCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg){
      if(!msg->detections.empty()){
        camera_pose.header = msg->header;
        camera_pose.pose = msg->detections[0].pose.pose.pose;
        if (listener.waitForTransform("base_link","camera_color_optical_frame",ros::Time(0),ros::Duration(3)))
        {
          try{
            listener.transformPose("base_link",camera_pose,base_pose);//将camera_color_optical_frame中的点变换到base_link中去
          }
          catch( tf::TransformException ex)
          {
            ROS_ERROR("transfrom exception : %s",ex.what());
          }
        }
        pub.publish(base_pose.pose);
        std::cout << "[TransForm] camera" << base_pose.pose << std::endl;
      }
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_transform");
  TransForm node;

  ros::Rate loop_rate(10);
  while (ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::shutdown();
  return 0;
}

