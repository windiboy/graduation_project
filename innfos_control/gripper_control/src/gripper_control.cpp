#include "SCServo.h"
#include "std_msgs/Int32.h"
#include "ros/ros.h"

class Gripper{
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub;
    std::string port_;
public:
    SMSBL sm;
    Gripper(){
        nh_.param<std::string>("servo_port", port_, "/dev/ttyUSB0");
        ROS_INFO("[gripper_control] serial: %s",port_.c_str());
        if(!sm.begin(115200, port_.c_str())){
            ROS_ERROR("[gripper_control] Failed to init smsbl motor!");
        }
        sub = nh_.subscribe("/gripper_control/int32",10,&Gripper::callback,this);
        sm.WritePosEx(1, 2, 4000, 100);//初始化 打开夹爪
    }
    void callback(const std_msgs::Int32::ConstPtr& msg){
        int pos;
        if(msg->data == 0){
            pos = 22;
        }
        else{
            pos = 761;
        }
        sm.WritePosEx(1, pos, 4000, 100);//舵机(ID1)以最高速度V=4000(50*80步/秒)，加速度A=100(100*100步/秒^2)，运行至位置
        usleep(800*1000);
        ROS_INFO("[gripper_control] write pos: %d",pos);
    }
};
int main(int argc, char** argv)
{
  ros::init(argc, argv, "gripper_control");
  Gripper node;

  ros::Rate loop_rate(10);
  while (ros::ok()){
    
    // std::cout<< "pos1 ="<< node.sm.ReadPos(1) <<std::endl;
    
    // node.sm.WritePosEx(1, 22, 4000, 100);//舵机(ID1)以最高速度V=80(50*80步/秒)，加速度A=100(100*100步/秒^2)，运行至P1=1800位置
    // std::cout<< "pos ="<<4095<<std::endl;
    // usleep(800*1000);//[(P1-P0)/(50*V)]*1000+[(50*V)/(A*100)]*1000

    // node.sm.WritePosEx(1, 761, 4000, 100);//舵机(ID1)以最高速度V=80(50*80步/秒)，加速度A=100(100*100步/秒^2)，运行至P0=2600位置
    // std::cout<< "pos ="<<0<<std::endl;
    // usleep(800*1000);//[(P1-P0)/(50*V)]*1000+[(50*V)/(A*100)]*1000

    ros::spinOnce();
    loop_rate.sleep();
  }
  node.sm.end();
  ros::shutdown();
  return 0;
}