# LeRobot Pico4 client commands

This document contains copy-and-run commands for the Pico4 teleoperator plugin.

The repository provides two command-line entry points:

```bash
lerobot-teleoperate-pico4
lerobot-record-pico4
```

It supports three teleoperator types:

| Teleoperator | Controller | Action dimensions | Intended robot |
| --- | --- | ---: | --- |
| `pico4` | One Pico controller | 10 | One Cartesian arm with a gripper |
| `bi_pico4` | Left and right Pico controllers | 20 | Two Cartesian arms with grippers |
| `pico4head` | Pico headset, enabled by the left X button | 9 | One 6-axis Cartesian arm without a gripper |


## Action schemas and compatibility

The teleoperator and robot action feature names must match exactly. The check is
performed before the robot is connected, and reports missing or extra fields.

### `pico4`

```text
tcp.x, tcp.y, tcp.z,
tcp.r1, tcp.r2, tcp.r3, tcp.r4, tcp.r5, tcp.r6,
gripper.pos
```

### `bi_pico4`

```text
left_tcp.x, left_tcp.y, left_tcp.z,
left_tcp.r1, left_tcp.r2, left_tcp.r3,
left_tcp.r4, left_tcp.r5, left_tcp.r6,
right_tcp.x, right_tcp.y, right_tcp.z,
right_tcp.r1, right_tcp.r2, right_tcp.r3,
right_tcp.r4, right_tcp.r5, right_tcp.r6,
left_gripper.pos, right_gripper.pos
```

### `pico4head`

`pico4head` is intended for a 6-axis robot without a gripper:

```text
tcp.x, tcp.y, tcp.z,
tcp.r1, tcp.r2, tcp.r3, tcp.r4, tcp.r5, tcp.r6
```

The numeric feature indexes sent by `pico4head` are:

```python
{
    "tcp.x": 0,
    "tcp.y": 1,
    "tcp.z": 2,
    "tcp.r1": 3,
    "tcp.r2": 4,
    "tcp.r3": 5,
    "tcp.r4": 6,
    "tcp.r5": 7,
    "tcp.r6": 8,
}
```

Robot type names are not hard-coded in the teleoperator. Any robot plugin can be
used when it exposes the exact action schema required by the selected teleoperator.

## Controller mapping

| Teleoperator | Input | Function |
| --- | --- | --- |
| `pico4` with right controller | Hold right grip | Enable arm motion; release to freeze the target |
| `pico4` with right controller | Right trigger | Control the gripper |
| `pico4` with right controller | Right A | Reset the robot to its configured initial position |
| `pico4` with left controller | Hold left grip | Enable arm motion; release to freeze the target |
| `pico4` with left controller | Left trigger | Control the gripper |
| `pico4` with left controller | Left X | Reset the robot to its configured initial position |
| `bi_pico4` | Hold each grip | Enable the corresponding arm independently |
| `bi_pico4` | Each trigger | Control the corresponding gripper |
| `bi_pico4` | Right A | Reset both arms |
| `pico4head` | Hold left X | Enable headset control of the 6-axis arm |
| `pico4head` | Release left X | Stop control and freeze the current target |
| `pico4head` | Right A | Reset the robot to its configured initial position |

Each time `pico4head` is enabled, the current headset pose and current robot TCP
pose are captured as fresh references. This prevents the robot from jumping to an
old headset offset after X is released and held again.

## Teleoperate a 6-axis RS arm as the head axis

This example uses `pico4head` with the consolidated gripperless RS follower in:

```text
https://github.com/xensedyl/lerobot-robot-seeed-b601-rt
```

Install the robot plugin and its Cartesian FK/IK dependency in the active environment:

```bash
git clone git@github.com:xensedyl/rebotarm_control_rt.git
cd rebotarm_control_rt
bash setup_env.sh --install

git clone git@github.com:xensedyl/lerobot-robot-seeed-b601-rt.git
cd lerobot-robot-seeed-b601-rt
pip install -e .
```

Run teleoperation:

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

Hold the left controller X button to move the RS arm with the Pico headset. Release
X to stop sending new headset targets.

## Teleoperate one B601 arm

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

By default, `pico4` uses the right controller. To use only the left controller:

```bash
    --teleop.use_left_controller=true \
    --teleop.use_right_controller=false
```

## Teleoperate two B601 arms

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

## Teleoperate TRON2

TRON2 uses the inverse gripper convention, so set `invert_gripper=true`:

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

Gripper conventions:

| Configuration | `gripper.pos=0` | `gripper.pos=1` | Robot |
| --- | --- | --- | --- |
| `invert_gripper=false` | Open | Closed | B601 |
| `invert_gripper=true` | Closed | Open | TRON2 |

## Record the 6-axis RS head arm

This robot has no camera or gripper, so this example disables video:

```bash
lerobot-record-pico4 \
    --robot.type=seeed_b601_rs_follower \
    --robot.port=can0 \
    --robot.id=rs_pico_head \
    --robot.can_adapter=socketcan \
    --robot.action_mode=cartesian \
    --robot.gripper_type=none \
    --teleop.type=pico4head \
    --teleop.id=pico4head \
    --dataset.repo_id=${HF_USER}/rs-pico4head-demo \
    --dataset.single_task="Move the RS head axis with the Pico headset" \
    --dataset.fps=60 \
    --dataset.num_episodes=10 \
    --dataset.episode_time_s=30 \
    --dataset.reset_time_s=10 \
    --dataset.video=false \
    --dataset.push_to_hub=false \
    --display_data=true
```

Remove `--dataset.push_to_hub=false` or set it to `true` when the dataset is ready
to upload.

## Record one B601 arm

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
    --dataset.single_task="Pick and place an object" \
    --dataset.fps=30 \
    --dataset.num_episodes=10 \
    --dataset.episode_time_s=30 \
    --dataset.reset_time_s=10 \
    --dataset.streaming_encoding=true \
    --dataset.vcodec=auto \
    --display_data=true
```

## Record two B601 arms

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
    --dataset.repo_id=${HF_USER}/b601-bimanual-pico4-demo \
    --dataset.single_task="Perform a bimanual manipulation task" \
    --dataset.fps=30 \
    --dataset.num_episodes=10 \
    --dataset.episode_time_s=30 \
    --dataset.reset_time_s=10 \
    --dataset.streaming_encoding=true \
    --dataset.vcodec=auto \
    --display_data=true
```

## Record TRON2

```bash
lerobot-record-pico4 \
    --robot.type=tron2 \
    --robot.robot_ip=10.192.1.2 \
    --robot.id=tron2 \
    --teleop.type=bi_pico4 \
    --teleop.id=bi_pico4 \
    --teleop.invert_gripper=true \
    --dataset.repo_id=${HF_USER}/tron2-pico4-demo \
    --dataset.single_task="Perform a bimanual manipulation task" \
    --dataset.fps=30 \
    --dataset.num_episodes=10 \
    --dataset.episode_time_s=30 \
    --dataset.reset_time_s=10 \
    --dataset.streaming_encoding=true \
    --dataset.vcodec=auto \
    --display_data=true
```

## Recording controls

| Input | Function |
| --- | --- |
| Right Arrow | Finish the current episode early and save it |
| Left Arrow | Discard and record the current episode again |
| Esc | Stop the recording session |
| Controller reset button | Return the robot to its configured initial position |

The reset button depends on the selected teleoperator; see the controller mapping
table above.

## Common options

### Teleoperation loop

| Option | Default | Description |
| --- | ---: | --- |
| `--fps` | `60` | Teleoperation loop frequency |
| `--teleop_time_s` | None | Optional maximum run time |
| `--display_data` | `false` | Display robot data with Rerun |
| `--display_ip` | None | Rerun target IP |
| `--display_port` | None | Rerun target port |

### Pico motion

| Option | Default | Description |
| --- | ---: | --- |
| `--teleop.pos_sensitivity` | `1.0` | Translation scale |
| `--teleop.ori_sensitivity` | `1.0` | Rotation scale |
| `--teleop.filter_window_size` | `1` | Pose smoothing window |
| `--teleop.position_jump_threshold` | `0.1` | Reject larger position jumps in metres |
| `--teleop.max_pos_velocity` | `1.0` | Maximum Cartesian position velocity |
| `--teleop.max_rot_velocity` | `6.28` | Maximum rotation velocity |

### Recording

| Option | Default | Description |
| --- | ---: | --- |
| `--dataset.fps` | `30` | Dataset frame rate |
| `--dataset.num_episodes` | `50` | Number of episodes |
| `--dataset.episode_time_s` | `60` | Maximum duration of each episode |
| `--dataset.reset_time_s` | `60` | Reset interval between episodes |
| `--dataset.video` | `true` | Store camera observations as video |
| `--dataset.push_to_hub` | `true` | Upload the completed dataset |
| `--dataset.streaming_encoding` | `true` | Encode video while recording |
| `--dataset.vcodec` | `libsvtav1` | Video codec; `auto` probes supported encoders |
| `--camera_stabilization_time_s` | `2` | Wait for cameras before each episode |
| `--resume` | `false` | Resume an existing local dataset |
