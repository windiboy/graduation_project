#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int32MultiArray.h"

#define RAD2DEG(x) ((x)*180./M_PI)

class PlatFormControl{
public:
    PlatFormControl(){
      pub = n.advertise<std_msgs::Int32MultiArray>("/platform_control", 100);
      sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &PlatFormControl::scanCallback,this);
      n.getParam("dis_min", dis_min);
      n.getParam("dis_max", dis_max);
      ROS_INFO("[front dis], %f, %f", dis_min, dis_max);
    }
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
        // int count = scan->scan_time / scan->time_increment;
        // ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
        // ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
        // float degree = RAD2DEG(scan->angle_min + scan->angle_increment * count/4);
        std_msgs::Int32MultiArray plat_msg;
        ROS_INFO("[front], %f, %f", 180.0, scan->ranges[0]);
        float dis = scan->ranges[0];
        if((dis > dis_min) && (dis < dis_max)){
            plat_msg.data.push_back(0);
            plat_msg.data.push_back(0);
        }else if(dis < dis_min){
            plat_msg.data.push_back(-500);
            plat_msg.data.push_back(-500);
        }else if(dis > dis_max){
            plat_msg.data.push_back(500);
            plat_msg.data.push_back(500);
        }
        pub.publish(plat_msg);
    }
    ros::Publisher pub;
    ros::Subscriber sub;
    ros::NodeHandle n;
    float dis_min,dis_max;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "platform_control");
  PlatFormControl center;

  ros::AsyncSpinner spinner(1);
  spinner.start();
  while(ros::ok()){
    
  }
  ros::shutdown();
  return 0;
}
