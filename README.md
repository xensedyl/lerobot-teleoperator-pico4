# LeRobot Pico4 Teleoperator

[中文版说明](./README.zh-CN.md)

Standalone LeRobot teleoperator plugin for Pico4 VR controller TCP teleoperation.

The single-arm teleoperator outputs Cartesian TCP actions:

- `tcp.x`, `tcp.y`, `tcp.z`
- `tcp.r1` ... `tcp.r6` using 6D rotation representation
- `gripper.pos` in `[0, 1]`

The bimanual teleoperator uses both Pico4 controllers and outputs 20 prefixed
actions ordered as left arm, right arm, left gripper, right gripper (matching the
TRON2 robot's `action_features` layout):

- `left_tcp.x`, `left_tcp.y`, `left_tcp.z`, `left_tcp.r1` ... `left_tcp.r6`
- `right_tcp.x`, `right_tcp.y`, `right_tcp.z`, `right_tcp.r1` ... `right_tcp.r6`
- `left_gripper.pos`, `right_gripper.pos`

Supported bimanual robots are `bi_seeed_b601_rt_follower` and `tron2`.

Install in the active LeRobot environment:

```bash
pip install -e /home/xense/rebot_lerobot/lerobot-teleoperator-pico4
```

Install LeRobot and the Pico4 SDK first. This package only installs the Pico4
teleoperator plugin and its command entry points.

Single-arm teleoperation example:

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

Single-arm recording example:

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

Bimanual teleoperation example:

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

Bimanual recording example:

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

TRON2 bimanual teleoperation example (TRON2 is inherently Cartesian, so
`--robot.action_mode` is not required; `--teleop.invert_gripper=true` matches
TRON2's gripper convention where `gripper.pos` high means open):

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

TRON2 bimanual recording example (same robot/teleop args as above, plus
`--dataset.*`; note the recording rate is `--dataset.fps`, not top-level `--fps`):

```bash
lerobot-record-pico4 \
  --robot.type=tron2 \
  --robot.robot_ip=10.192.1.2 \
  --robot.id=tron2 \
  --teleop.type=bi_pico4 \
  --teleop.id=bi_pico4 \
  --teleop.invert_gripper=true \
  --dataset.repo_id=${HF_USER}/tron2-pico4-demo \
  --dataset.single_task="Bimanual TRON2 pick and place" \
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

Notes:
- `--dataset.repo_id` and `--dataset.single_task` are required. Use the
  `username/dataset_name` form for `repo_id`.
- Set `--dataset.push_to_hub=false` to keep the dataset local
  (`~/.cache/huggingface/lerobot/<repo_id>`, or override with `--dataset.root=/path`).
  Use `true` to upload, after `hf auth login`.
- During recording: **→** ends the current episode and enters reset, **←**
  re-records the episode, **ESC** stops recording. Between episodes there is a
  `--dataset.reset_time_s` reset phase; the right-controller **A** button returns
  both arms to their initial pose (recorded too).
- The recorded action layout is left arm, right arm, left gripper, right gripper (20-D).
- Do a quick `--dataset.num_episodes=1 --dataset.push_to_hub=false` dry run first.

For two B601 arms, check the current serial ports before running:

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
```

Then pass the actual B601 controller ports through `--robot.left_port` and
`--robot.right_port`. The Pico4 SDK must see both controllers; the left
controller drives `left_*` actions and the right controller drives `right_*`
actions. The right-controller A button resets both arms to their initial poses.

If push to huggingface, first check authentication:

```bash
hf auth whoami
```

If not login in:

```bash
hf auth login
```

These commands are provided by the plugin and leave LeRobot's built-in
`lerobot-teleoperate` and `lerobot-record` scripts unchanged.

The Pico4 SDK Python module `xensevr_pc_service_sdk` must already be installed
in the same environment.
