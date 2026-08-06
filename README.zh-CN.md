# LeRobot Pico4 遥操作插件

[English README](./README.md)

这是一个独立的 LeRobot Pico4 遥操作插件，安装后通过 LeRobot 的第三方插件注册机制提供：

- `--teleop.type=pico4`
- `--teleop.type=bi_pico4`
- `--teleop.type=pico4head`
- `--teleop.type=bi_pico4_head`

单臂 teleoperator 输出笛卡尔 TCP 动作：

- `tcp.x`, `tcp.y`, `tcp.z`
- `tcp.r1` ... `tcp.r6`，使用 6D rotation 表示姿态
- `gripper.pos`，范围是 `[0, 1]`

`pico4head` 使用头显位姿，输出不带夹爪的 9 维 TCP action。长按左手柄 X 键
启用遥操作，松开 X 键后保持当前目标。

双臂 teleoperator 使用 Pico4 左右两个手柄，输出 20 个带前缀的动作，顺序为
左臂、右臂、左夹爪、右夹爪（与 TRON2 机器人的 `action_features` 布局一致）：

- `left_tcp.x`, `left_tcp.y`, `left_tcp.z`, `left_tcp.r1` ... `left_tcp.r6`
- `right_tcp.x`, `right_tcp.y`, `right_tcp.z`, `right_tcp.r1` ... `right_tcp.r6`
- `left_gripper.pos`, `right_gripper.pos`

`bi_pico4_head` 是与机器人型号无关的“双臂 + 主动头部”遥操作器。它在一次
XenseVR SDK 连接里同时读取左右手柄和头显，输出 29 维 action：上面的双臂
20 维，加 `head_tcp.x/y/z/r1-r6` 9 维。左手柄 X 按住时头部跟随，右手柄 A
复位双臂和头部。

遥操作层不会导入、白名单判断或分支处理任何机器人品牌。兼容 Robot 只需要：

```text
action_features 精确包含 29 个通用键
get_current_tcp_pose_quat() -> (left_pose, right_pose, head_pose)
send_action(action_29d)
reset_to_initial_position()
```

B601 示例统一使用
`/home/xense/rebot_lerobot/lerobot-robot-seeed-b601-rt`。6 轴头部机械臂使用
`seeed_b601_rs_follower`，RT 双臂使用 `bi_seeed_b601_rt_follower`；TRON2
同样支持 `bi_pico4`。

## 安装

Pico SDK 和本插件必须安装在同一个 mamba 环境。先在当前终端激活目标环境，
因为 `setup_env.sh --install` 会安装到当前已激活的环境：

```bash
mamba activate <lerobot-env>
git clone git@github.com:xensedyl/Xense-Pico-Teleop-Interface.git
cd Xense-Pico-Teleop-Interface
bash setup_env.sh --install

cd lerobot-teleoperator-pico4
pip install -e .
python -c "import xensevr_pc_service_sdk; print('Pico SDK is available')"
```

如果 SDK 仓库已经存在，跳过 `git clone`，直接进入已有仓库执行
`bash setup_env.sh --install`。

只有 `pico4`、`bi_pico4`、`pico4head` 或 `bi_pico4_head` 执行连接时才检查
SDK。缺少 SDK 时，
对应 teleoperator 会报错并打印上面的安装命令；安装包和导入配置不要求 SDK。

非夕双臂 + Seeed RS 头部只是一个可跑通的机器人侧示例：

```bash
lerobot-teleoperate \
  --robot.type=bi_flexiv_rizon4_rt_head \
  --robot.bi_mount_type=forward-04 \
  --robot.head_port=can0 \
  --teleop.type=bi_pico4_head \
  --teleop.head_pos_sensitivity=1 \
  --teleop.head_ori_sensitivity=1 \
  --fps=30 \
  --display_data=true
```

B601 示例还需要在同一环境中安装最终 Robot 仓库和 FK/IK 包：

```bash
git clone git@github.com:xensedyl/rebotarm_control_rt.git
cd rebotarm_control_rt
bash setup_env.sh --install

git clone git@github.com:xensedyl/lerobot-robot-seeed-b601-rt.git
cd lerobot-robot-seeed-b601-rt
pip install -e .
```

## 遥操作

使用头显遥操作 6 轴、无夹爪的 RS 机械臂：

```bash
lerobot-teleoperate-pico4 \
  --robot.type=seeed_b601_rs_follower \
  --robot.port=can0 \
  --robot.id=rs_pico_head \
  --robot.can_adapter=socketcan \
  --robot.action_mode=cartesian \
  --robot.gripper_type=none \
  --teleop.type=pico4head \
  --teleop.id=pico4head \
  --fps=60 \
  --display_data=true
```

RobStride URDF 已安装在 `rebotarm_control_rt` 包内，不需要额外传 URDF 路径。

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

TRON2 双臂遥操作示例（TRON2 本身就是笛卡尔控制，无需 `--robot.action_mode`；
`--teleop.invert_gripper=true` 用于匹配 TRON2 的夹爪方向，即 `gripper.pos` 越大越开）：

```bash
lerobot-teleoperate-pico4 \
  --robot.type=tron2 \
  --robot.robot_ip=10.192.1.2 \
  --robot.id=tron2 \
  --teleop.type=bi_pico4 \
  --teleop.id=bi_pico4 \
  --teleop.invert_gripper=true \
  --fps=30 \
  --display_data=true
```

TRON2 双臂数据采集示例（机器人/手柄参数和上面遥操作一样，多了 `--dataset.*`；
注意采集帧率是 `--dataset.fps`，不是顶层 `--fps`）：

```bash
lerobot-record-pico4 \
  --robot.type=tron2 \
  --robot.robot_ip=10.192.1.2 \
  --robot.id=tron2 \
  --teleop.type=bi_pico4 \
  --teleop.id=bi_pico4 \
  --teleop.invert_gripper=true \
  --dataset.repo_id=${HF_USER}/tron2-pico4-demo \
  --dataset.single_task="双臂 TRON2 抓取放置" \
  --dataset.num_episodes=5 \
  --dataset.fps=30 \
  --dataset.episode_time_s=60 \
  --dataset.reset_time_s=30 \
  --dataset.streaming_encoding=true \
  --dataset.vcodec=auto \
  --resume=false \
  --dataset.push_to_hub=true \
  --display_data=false
```

--dataset.encoder_threads=1 \
--camera_stabilization_time_s=1 \

说明：
- `--dataset.repo_id`、`--dataset.single_task` 必填，`repo_id` 用 `用户名/数据集名` 格式。
- `--dataset.push_to_hub=false` 只存本地（`~/.cache/huggingface/lerobot/<repo_id>`，
  或用 `--dataset.root=/路径` 指定）；设 `true` 则上传，需先 `hf auth login`。
- 采集中键盘控制：**→** 提前结束当前 episode 并进入复位，**←** 重录当前 episode，
  **ESC** 停止录制。每个 episode 之间有 `--dataset.reset_time_s` 复位阶段；右手柄
  **A 键**让双臂回到初始位（复位过程也会被录进数据）。
- 记录的动作维度为 左臂、右臂、左夹爪、右夹爪（20 维）。
- 先用 `--dataset.num_episodes=1 --dataset.push_to_hub=false` 跑一条验证流程。

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
