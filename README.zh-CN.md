# LeRobot Pico4 遥操作插件

[English README](./README.md)

这是一个独立的 LeRobot Pico4 遥操作插件，安装后通过 LeRobot 的第三方插件注册机制提供：

- `--teleop.type=pico4`
- `--teleop.type=bi_pico4`

单臂 teleoperator 输出笛卡尔 TCP 动作：

- `tcp.x`, `tcp.y`, `tcp.z`
- `tcp.r1` ... `tcp.r6`，使用 6D rotation 表示姿态
- `gripper.pos`，范围是 `[0, 1]`

双臂 teleoperator 使用 Pico4 左右两个手柄，输出带前缀的动作：

- `left_tcp.x`, `left_tcp.y`, `left_tcp.z`, `left_tcp.r1` ... `left_tcp.r6`, `left_gripper.pos`
- `right_tcp.x`, `right_tcp.y`, `right_tcp.z`, `right_tcp.r1` ... `right_tcp.r6`, `right_gripper.pos`

## 安装

在 LeRobot 使用的 Python 环境里安装：

```bash
pip install -e /home/xense/rebot_lerobot/lerobot-teleoperator-pico4
```

先安装 LeRobot 和 Pico4 SDK。这个包只安装 Pico4 teleoperator 插件和命令入口，
不负责安装 LeRobot、B601 robot 插件或 Pico4 SDK。

Pico4 SDK 的 Python 模块 `xensevr_pc_service_sdk` 需要提前装在同一个环境里。

## 遥操作

Pico4 输出的是 TCP 目标，所以 B601 需要用笛卡尔模式：

```bash
lerobot-teleoperate-pico4 \
  --robot.type=seeed_b601_rt_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.id=follower1 \
  --robot.can_adapter=damiao \
  --robot.action_mode=cartesian \
  --teleop.type=pico4 \
  --teleop.id=pico4 \
  --fps=100 \
  --display_data=true
```

双臂遥操作：

```bash
lerobot-teleoperate-pico4 \
  --robot.type=bi_seeed_b601_rt_follower \
  --robot.left_port=/dev/ttyACM0 \
  --robot.right_port=/dev/ttyACM1 \
  --robot.id=bi_follower \
  --robot.can_adapter=damiao \
  --robot.action_mode=cartesian \
  --teleop.type=bi_pico4 \
  --teleop.id=bi_pico4 \
  --fps=100 \
  --display_data=true
```

## 采集数据

```bash
lerobot-record-pico4 \
  --robot.type=seeed_b601_rt_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.id=follower1 \
  --robot.can_adapter=damiao \
  --robot.action_mode=cartesian \
  --teleop.type=pico4 \
  --teleop.id=pico4 \
  --dataset.repo_id=${HF_USER}/b601-pico4-demo \
  --dataset.single_task="Teleoperate B601 with Pico4" \
  --dataset.num_episodes=1 \
  --dataset.fps=30 \
  --dataset.episode_time_s=600 \
  --dataset.reset_time_s=120 \
  --resume=false \
  --dataset.push_to_hub=true \
  --display_data=false
```

双臂采集数据：

```bash
lerobot-record-pico4 \
  --robot.type=bi_seeed_b601_rt_follower \
  --robot.left_port=/dev/ttyACM0 \
  --robot.right_port=/dev/ttyACM1 \
  --robot.id=bi_follower \
  --robot.can_adapter=damiao \
  --robot.action_mode=cartesian \
  --teleop.type=bi_pico4 \
  --teleop.id=bi_pico4 \
  --dataset.repo_id=${HF_USER}/b601-bi-pico4-demo \
  --dataset.single_task="Teleoperate dual B601 with Pico4" \
  --dataset.num_episodes=1 \
  --dataset.fps=30 \
  --dataset.episode_time_s=600 \
  --dataset.reset_time_s=120 \
  --resume=false \
  --dataset.push_to_hub=true \
  --display_data=false
```

双臂运行前先确认当前串口：

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
```

然后把实际两个 B601 控制器端口分别传给 `--robot.left_port` 和
`--robot.right_port`。`bi_pico4` 中左手柄控制 `left_*` 动作，右手柄控制
`right_*` 动作。右手柄 A 键会让双臂同时复位到启动时的初始位置。

如果要推送到 Hugging Face，请先检查认证状态：

```bash
hf auth whoami
```

如果尚未登录：

```bash
hf auth login
```

## 说明

这个包提供独立命令 `lerobot-teleoperate-pico4` 和 `lerobot-record-pico4`，
不需要修改 LeRobot 主仓库的 `lerobot-teleoperate` / `lerobot-record` 脚本。

控制循环逻辑是：

- 插件声明 `requires_current_tcp_pose=True`
- robot 提供 `get_current_tcp_pose_quat()`
- `lerobot-teleoperate-pico4` 在 `get_action()` 前把当前 TCP 同步给 Pico4

因此 Pico4 插件可以独立维护，B601 内部负责把 TCP action 逆解成关节目标。
