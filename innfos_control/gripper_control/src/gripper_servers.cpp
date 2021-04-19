//action服务端的相关定义，请加入到驱动节点的头文件中
#include "actionlib/server/action_server.h"
//action服务端的目标控制句柄定义，与接收的目标相关联后，可以用来实现action的信息反馈等操作
#include "actionlib/server/server_goal_handle.h"
#include "control_msgs/GripperCommand.h"
#include "ros/ros.h"
#include "SCServo.h"

#include <iostream>
#include <vector>
#include <map>

using namespace std;

class GripperService
{
protected:

    ros::NodeHandle nh_;
    //定义action服务端
    actionlib::ActionServer<control_msgs::GripperCommand>  as_;
    //定义action服务端目标控制句柄
    actionlib::ServerGoalHandle<control_msgs::GripperCommand> goal_handle_;
    //用来反馈action目标的执行情况，客户端由此可以得知服务端是否执行成功了
    control_msgs::GripperCommand result_;

    std::string action_name_;

public:
    SMSBL sm;
    GripperService(std::string name) :
            as_(nh_, name, boost::bind(&GripperService::goalCB, this, _1), false),
            action_name_(name)
    {
        as_.start();
        if(!sm.begin(115200, port_.c_str())){
            ROS_ERROR("[gripper_service] Failed to init smsbl motor!");
        }
    }

    void goalCB( actionlib::ServerGoalHandle<control_msgs::GripperCommand> gh)
    {
        //疑点1：什么要将形参复制一份？
        //因为我们要修改这个参数，但又不#include "../include/MotorDriver.h"能直接改形参所指向的那个目标对象，那样做会破坏原始数据，是编程所不推荐的，这里复制一个副本，目的就是要修改这个数据而不破坏原始数据。
        actionlib::ActionServer<control_msgs::GripperCommand>::Goal goal = *gh.getGoal();    //make a copy that we can modify

        //将之前定义的句柄与传入的action目标绑定在一起，
        goal_handle_ = gh;

        //excute
        int pos;
        if(goal.position == 0){
            pos = 22;
        }
        else{
            pos = 761;
        }
        sm.WritePosEx(1, pos, 4000, 100);//舵机(ID1)以最高速度V=4000(50*80步/秒)，加速度A=100(100*100步/秒^2)，运行至位置
        usleep(800*1000);
        ROS_INFO("[gripper_control] write pos: %d",pos);
        //告诉客户端成功了
        goal_handle_.setAccepted();
        result_.error_code = result_.SUCCESSFUL;
        goal_handle_.setSucceeded(result_);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "gripper_service");

    GripperService gripperservice("gripper_action/GripperCommand");
    ros::spin();

    return 0;
}