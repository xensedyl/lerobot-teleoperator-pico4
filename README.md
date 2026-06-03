# LeRobot Pico4 Teleoperator

[中文版说明](./README.zh-CN.md)

Standalone LeRobot teleoperator plugin for Pico4 VR controller TCP teleoperation.

The teleoperator outputs Cartesian TCP actions:

- `tcp.x`, `tcp.y`, `tcp.z`
- `tcp.r1` ... `tcp.r6` using 6D rotation representation
- `gripper.pos` in `[0, 1]`

Install in the active LeRobot environment:

```bash
pip install -e /home/xense/rebot_lerobot/lerobot-teleoperator-pico4
```

Install LeRobot and the Pico4 SDK first. This package only installs the Pico4
teleoperator plugin and its command entry points.

Example:

```bash
lerobot-teleoperate-pico4 \
  --robot.type=seeed_b601_rt_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.id=follower1 \
  --robot.can_adapter=damiao \
  --robot.action_mode=cartesian \
  --teleop.type=pico4 \
  --teleop.id=pico4 \
  --fps=100
```

Recording example:

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
  --dataset.fps=30
```

This command is provided by the plugin and leaves LeRobot's built-in
`lerobot-teleoperate` script unchanged.

The Pico4 SDK Python module `xensevr_pc_service_sdk` must already be installed
in the same environment.
