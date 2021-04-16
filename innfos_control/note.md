# 毕业设计

## 2021年4月10日

机械臂网口信息inet 192.168.1.230  netmask 255.255.255.0  broadcast 192.168.1.1

```python
sudo apt install zsh
sh -c "$(curl -fsSL <https://raw.github.com/robbyrussell/oh-my-zsh/master/tools/install.sh>)"
git clone git://github.com/zsh-users/zsh-autosuggestions $ZSH_CUSTOM/plugins/zsh-autosuggestions
git clone <https://github.com/zsh-users/zsh-syntax-highlighting.git> ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting

sudo apt install ros-melodic-ar-track-alvar
sudo apt install ros-melodic-realsense2-camera
sudo apt install openssh-server
```

***

## 2021年4月12日

1. group manipulator not found
    - 默认的规划组的名字是manipulator 而我们的是gluon 需要再handeye_server_robot.py中指定
2. easy_handeye start pose faile
    - 需要注释handeye_robot.py_check_target_poses 的两行

    ```python
    # if len(plan.joint_trajectory.points) == 0 or CalibrationMovements._is_crazy_plan(plan, joint_limits):
    #     return False
    ```

3. 标定到某个点规划不出路径，导致程序退出
    - TODO：
    - 暂时可以跳过几个点算出tf
    - 估计是规划失败的情况easy_handeye并不能获取到，所以程序崩溃了

***

## 2021年4月13日

1. calibrate done
    - 需要用aruco_ros来校准，因为Apriltag没有camera_link的变换
    - aruco_calibration.launch
    - 校准完的文件在~/.ros/easy_handeyey/ 可以通过calibrate_backup.sh进行备份
    - TODO：目前还是一些误差
2. follow done
    - 把apriltag识别到的pose变换到base link下，控制机械臂的跟随
    - 初步完成了跟随，但是因为校准的误差，mark和机械臂有一些误差
3. TODO
    - 因为plan求解默认是5s，所以规划失败的时候会卡住5s，明天看看怎么解决
    - 另一方面可以研究一下，用路径点规划会不会好一些，可以设置末端的位姿误差，可以提高求解率

## 2021年4月14日

1. gripper done
    - 机械爪驱动搞定，并可以通过话题控制
2. pick demo
    - 用路径规划控制可以实现抓取，不过末端的抓取姿态还是个大问题，因为相机只能给出位置，不能给出抓取姿态
    - 四元数的理解太抽象了，看看怎么用欧拉角转化理解一下
3. 规划求解器
    - 目前把默认的kdl求解器换成了trac_ik，求解速度提升了，不过trac_ik每次解析不唯一
    - 有时间的话看看IKFast

## 2021年4月16日

1. end_effector_rpy_publisher
    - 写了一个把末端姿态四元数转化为欧拉角的node，方便理解和调试
    - 按理说四元数表示的是旋转，和位置无关，但是相同四元数在不同位置的表现却不一样，很奇怪。