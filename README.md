# Motion_control_accurate
输入： map 坐标系下的idea_point，map系下的当前位姿态
目标： 写成action

## action_of_motion

微调流程通过 `/move_to_pose` action 启动，目标输入为 map 坐标系下的
`x, y, yaw_deg`。节点订阅 `/odin1/relocation`
(`geometry_msgs/PoseStamped`)，并发布 `std_msgs/Float32MultiArray` 到
`/t0x0101_pid`，数组顺序为 `[vx_body, vy_body, wz]`。

`pose_adjust_mode` 控制位姿微调方式：

- `simultaneous`：默认模式，直接边平移边旋转修正。使用独立的
  `pose_x_pid`、`pose_y_pid`、`pose_yaw_pid`，以及
  `pose_timeout_sec`、`pose_max_linear_speed`、
  `pose_min_linear_speed`、`pose_max_yaw_angular_speed`、
  `pose_min_yaw_angular_speed`。
- `staged`：分步模式，先原地旋转到目标 yaw，再平移并小幅修正 yaw。
  使用原有 `yaw_pid`、`x_pid`、`y_pid`、`xy_yaw_pid` 参数。

`simultaneous` 模式下，位置误差进入 `position_tolerance` 且 yaw 误差进入
`yaw_tolerance_deg` 后 action 才会返回成功。

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

查看最近生成的 PID 调试图：

```bash
./src/action_of_motion/scripts/plot_pid_debug.sh
```

debug 模式下每次 action 结束会自动生成 matplotlib 曲线图：

```bash
ls output/pid_debug
```
