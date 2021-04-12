inet 192.168.1.230  netmask 255.255.255.0  broadcast 192.168.1.1
sh -c "$(curl -fsSL <https://raw.github.com/robbyrussell/oh-my-zsh/master/tools/install.sh>)"
git clone git://github.com/zsh-users/zsh-autosuggestions $ZSH_CUSTOM/plugins/zsh-autosuggestions
git clone <https://github.com/zsh-users/zsh-syntax-highlighting.git> ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting

sudo apt install ros-melodic-ar-track-alvar

## 2021年4月12日

- group manipulator not found
默认的规划组的名字是manipulator 而我们的是gluon 需要再handeye_server_robot.py中指定
- easy_handeye start pose faile
需要注释handeye_robot.py _check_target_poses 的两行

```python
# if len(plan.joint_trajectory.points) == 0 or CalibrationMovements._is_crazy_plan(plan, joint_limits):
#     return False
```

- 标定到某个点规划不出路径，导致程序退出
