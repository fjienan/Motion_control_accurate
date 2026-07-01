# Motion_control_accurate
输入： map 坐标系下的idea_point，map系下的当前位姿态
目标： 写成action

## action_of_motion

微调流程通过 `/move_to_pose` action 启动，目标输入为 map 坐标系下的
`x, y, yaw_deg`，PID 档位 `pid_profile`，以及可选速度覆盖值
`max_vel` / `max_wz`。节点订阅 `/odin1/relocation`
(`geometry_msgs/PoseStamped`)，并发布 `std_msgs/Float32MultiArray` 到
`/t0x0101_pid`，数组顺序为 `[vx_body, vy_body, wz]`。

节点只保留 simultaneous 微调方式：边平移边旋转修正。Action goal 中的
`pid_profile` 用来选择两套独立 PID 参数：

- `0` / `slow`：缓慢型，`slow_profile.max_linear_speed` 默认为 `0.8`。
- `1` / `fast`：快速型，`fast_profile.max_linear_speed` 默认为 `2.0`。

`max_vel` 单位为 m/s，默认 `0.0` 表示沿用所选 profile 的原始
`max_linear_speed`；当 `max_vel > 0.0` 时，仅覆盖本次 goal 的最大线速度，
`max_wz` 单位为 rad/s，默认 `0.0` 表示沿用所选 profile 的原始
`max_yaw_angular_speed`；当 `max_wz > 0.0` 时，仅覆盖本次 goal 的最大 yaw
角速度。

每套 profile 都有独立的 `along_pid`、`cross_pid`、`yaw_pid`，以及独立的
线速度和角速度限制。控制器会按 action 开始时的当前位置到目标点建立 map
坐标系直线路径，分别控制沿线路径误差和横向偏差；位置误差进入
`position_tolerance` 且 yaw 误差进入 `yaw_tolerance_deg` 后 action 才会返回成功。

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
./src/action_of_motion/scripts/send_move_goal.sh 1.0 0.5 90.0 slow
./src/action_of_motion/scripts/send_move_goal.sh 1.0 0.5 90.0 fast
./src/action_of_motion/scripts/send_move_goal.sh 1.0 0.5 90.0 fast 0.4 0.8
```

直接使用 `ros2 action send_goal`：

```bash
ros2 action send_goal /move_to_pose action_of_motion_interfaces/action/MoveToPose \
  "{x: 1.0, y: 0.5, yaw_deg: 90.0, pid_profile: 0, max_vel: 0.4, max_wz: 0.8}" --feedback
```

低延迟调用方式：

`send_move_goal.sh` 和 `ros2 action send_goal` 适合手动调试，但它们每次都会
启动一个新的 ROS2 CLI 进程并重新做 action discovery，不适合作为低延迟业务调用。
在其他常驻 Python 程序里，推荐复用同一个 `MoveToPoseClient` 实例：

```python
import rclpy
from rclpy.node import Node

from action_of_motion.move_to_pose_client import MoveToPoseClient


rclpy.init()
node = Node('move_to_pose_user')
client = MoveToPoseClient(node)
client.wait_for_server()

result1 = client.send_goal(1.0, 0.5, 90.0, 'slow')
result2 = client.send_goal(2.0, 0.0, 0.0, 'fast', max_vel=0.4, max_wz=0.8)

node.destroy_node()
rclpy.shutdown()
```

同一个 `client` 可以多次调用 `send_goal()`，不需要每次重新创建 client 或重新等待
server。如果调用方已经有自己的 ROS2 node，就直接把已有 node 传给
`MoveToPoseClient`。如果调用方是 C++ 程序，也建议使用常驻
`rclcpp_action::Client`，不要从程序里反复拉起 shell 命令。

查看最近生成的 PID 调试图：

```bash
./src/action_of_motion/scripts/plot_pid_debug.sh
```

debug 模式下每次 action 结束会自动生成 matplotlib 曲线图，文件名和标题里会包含
`slow` 或 `fast`，便于区分是哪套 PID 参数：

```bash
ls output/pid_debug
```
