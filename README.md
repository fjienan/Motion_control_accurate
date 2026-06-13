# Motion_control_accurate
输入： map 坐标系下的idea_point，map系下的当前位姿态
目标： 写成action

## action_of_motion

微调流程通过 `/move_to_pose` action 启动，目标输入为 map 坐标系下的
`x, y, yaw_deg`。节点订阅 `/odin1/relocation`
(`geometry_msgs/PoseStamped`)，并发布 `std_msgs/Float32MultiArray` 到
`/t0x0101_pid`，数组顺序为 `[vx_body, vy_body, wz]`。

构建：

```bash
colcon build --packages-select action_of_motion_interfaces action_of_motion
source install/setup.bash
```

启动节点：

```bash
ros2 run action_of_motion motion_action_node --ros-args \
  --params-file src/action_of_motion/config/param.yaml
```

或者使用 launch 启动：

```bash
ros2 launch action_of_motion motion_action.launch.py
```

指定其他参数文件：

```bash
ros2 launch action_of_motion motion_action.launch.py \
  params_file:=/path/to/param.yaml
```

发送调试 goal：

```bash
./src/action_of_motion/scripts/send_move_goal.sh 1.0 0.5 90.0
```
