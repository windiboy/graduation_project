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
