# LeRobot Pico4 Teleoperator

[中文版说明](./README.zh-CN.md)

Standalone LeRobot teleoperator plugin for Pico4 VR controller TCP teleoperation.

The single-arm teleoperator outputs Cartesian TCP actions:

- `tcp.x`, `tcp.y`, `tcp.z`
- `tcp.r1` ... `tcp.r6` using 6D rotation representation
- `gripper.pos` in `[0, 1]`

The bimanual teleoperator uses both Pico4 controllers and outputs prefixed actions:

- `left_tcp.x`, `left_tcp.y`, `left_tcp.z`, `left_tcp.r1` ... `left_tcp.r6`, `left_gripper.pos`
- `right_tcp.x`, `right_tcp.y`, `right_tcp.z`, `right_tcp.r1` ... `right_tcp.r6`, `right_gripper.pos`

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
  --resume=false \
  --dataset.push_to_hub=true \
  --display_data=false
```

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
