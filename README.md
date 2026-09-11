# LeRobot Pico4 Teleoperator

[中文版说明](./README.zh-CN.md)

Standalone LeRobot teleoperator plugin for Pico4 controller and headset TCP
teleoperation.

The single-arm teleoperator outputs Cartesian TCP actions:

- `tcp.x`, `tcp.y`, `tcp.z`
- `tcp.r1` ... `tcp.r6` using 6D rotation representation
- `gripper.pos` in `[0, 1]`

The `pico4head` teleoperator uses the headset pose and outputs a 9D TCP action
without a gripper. Hold the left controller X button to enable motion and release
it to freeze the current target.

The bimanual teleoperator uses both Pico4 controllers and outputs 20 prefixed
actions ordered as left arm, right arm, left gripper, right gripper (matching the
TRON2 robot's `action_features` layout):

- `left_tcp.x`, `left_tcp.y`, `left_tcp.z`, `left_tcp.r1` ... `left_tcp.r6`
- `right_tcp.x`, `right_tcp.y`, `right_tcp.z`, `right_tcp.r1` ... `right_tcp.r6`
- `left_gripper.pos`, `right_gripper.pos`

`bi_pico4_head` is the robot bimanual-plus-head teleoperator. It
combines both controller streams and the headset stream through one SDK
connection, and outputs 29 actions: the 20 bimanual fields above followed by
`head_tcp.x/y/z/r1-r6`. Hold left X to move the head; right A resets the
complete robot.

It does not import, whitelist, or branch on a robot brand. A compatible robot
only needs the exact 29 action keys and these control methods:

```text
get_current_tcp_pose_quat() -> (left_pose, right_pose, head_pose)
send_action(action_29d)
reset_to_initial_position()
```

Supported B601 examples use the consolidated
`/home/xense/rebot_lerobot/lerobot-robot-seeed-b601-rt` plugin. The 6-axis head
arm uses `seeed_b601_rs_follower`; bimanual RT uses
`bi_seeed_b601_rt_follower`. TRON2 is also supported by `bi_pico4`.

Install the Pico SDK and this plugin in the same active mamba environment. Run
`mamba activate` in the current terminal before invoking the SDK installer,
because `setup_env.sh --install` installs into the currently active environment:

```bash
mamba activate <lerobot-env>
git clone git@github.com:xensedyl/Xense-Pico-Teleop-Interface.git
cd Xense-Pico-Teleop-Interface
bash setup_env.sh --install

cd lerobot-teleoperator-pico4
pip install -e .
python -c "import xensevr_pc_service_sdk; print('Pico SDK is available')"
```

If the SDK repository is already cloned, skip `git clone` and run
`bash setup_env.sh --install` from the existing checkout.

The SDK is checked only when `pico4`, `bi_pico4`, `pico4head`, or
`bi_pico4_head` connects. If it
is missing, the selected teleoperator raises an error containing the installation
commands above. Package installation and configuration imports do not require
the SDK.

One compatible robot-side example is dual Flexiv arms plus a Seeed RS head in
`lerobot-xense`:

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

For the B601 examples, install the consolidated robot plugin and build its
packaged FK/IK dependency in the same environment:

```bash
git clone git@github.com:xensedyl/rebotarm_control_rt.git
cd rebotarm_control_rt
bash setup_env.sh --install

git clone git@github.com:xensedyl/lerobot-robot-seeed-b601-rt.git
cd lerobot-robot-seeed-b601-rt
pip install -e .
```

6-axis RS head teleoperation example (no gripper):

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

The RobStride URDF is installed inside `rebotarm_control_rt`; no URDF path is
required in this command.

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

TRON2 RT bimanual teleoperation example (300 Hz native loop with 30 Hz Pico4
waypoints; `--teleop.invert_gripper=true` matches
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

```bash
lerobot-teleoperate-pico4 \
  --robot.type=tron2_rt \
  --robot.robot_ip=10.192.1.2 \
  --robot.robot_port=5000 \
  --robot.control_mode=cartesian \
  --robot.control_hz=300 \
  --robot.use_grippers=true \
  --robot.use_head=true \
  --robot.reset_on_disconnect=true \
  --robot.gripper.control_mode=mit \
  --teleop.type=bi_pico4_head \
  --teleop.id=bi_pico4_head \
  --teleop.invert_gripper=true \
  --fps=30 \
  --display_data=true
```

```bash
lerobot-teleoperate-pico4 \
  --robot.type=tron2_rt \
  --robot.robot_ip=10.192.1.2 \
  --robot.robot_port=5000 \
  --robot.control_mode=cartesian \
  --robot.control_hz=300 \
  --robot.use_grippers=true \
  --robot.use_head=true \
  --robot.reset_on_disconnect=true \
  --robot.camera_host=10.192.1.4 \
  --robot.gripper.type=taccap_follower \
  --robot.gripper.remote_base_url=http://10.192.1.4:8765 \
  --robot.gripper.remote_auto_enable=true \
  --robot.gripper.auto_discover_cameras=true \
  --robot.gripper.enable_tactile=true \
  --robot.gripper.control_mode=mit \
  --robot.gripper.remote_timeout_s=2 \
  --teleop.type=bi_pico4_head \
  --teleop.id=bi_pico4_head \
  --teleop.invert_gripper=true \
  --fps=30 \
  --display_data=true
```

`--robot.gripper.control_mode=mit` selects the TacCap MIT impedance command;
use `position` for the bounded firmware position command. The setting is sent
to the `.4` service when each follower connects. The service Web page can also
change the left and right modes independently while it is running.


For TRON2 RT, `--fps=60` sends 60 Hz waypoints when the loop can sustain it.
The native 300 Hz controller measures the actual arrival interval and chooses
the interpolation samples automatically; no second robot command frequency is
needed.

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

```bash
lerobot-record-pico4 \
  --robot.type=tron2_rt \
  --robot.robot_ip=10.192.1.2 \
  --robot.robot_port=5000 \
  --robot.control_mode=cartesian \
  --robot.control_hz=300 \
  --robot.use_grippers=true \
  --robot.use_head=true \
  --robot.reset_on_disconnect=true \
  --teleop.type=bi_pico4_head \
  --teleop.id=bi_pico4_head \
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
  --display_data=false \
  --resume=false
```

```bash
lerobot-record-pico4 \
  --robot.type=tron2_rt \
  --robot.robot_ip=10.192.1.2 \
  --robot.robot_port=5000 \
  --robot.control_mode=cartesian \
  --robot.control_hz=300 \
  --robot.use_grippers=true \
  --robot.use_head=true \
  --robot.reset_on_disconnect=true \
  --robot.use_tool_calibration=true \
  --robot.camera_host=10.192.1.4 \
  --robot.taccap_remote_url=http://10.192.1.4:8765 \
  --robot.taccap_remote_auto_enable=true \
  --robot.gripper.type=taccap_follower \
  --robot.gripper.auto_discover_cameras=false \
  --robot.gripper.enable_tactile=true \
  --robot.gripper.remote_timeout_s=2 \
  --teleop.type=bi_pico4_head \
  --teleop.id=bi_pico4_head \
  --teleop.invert_gripper=true \
  --dataset.repo_id=xensedyl/tron2rt-pico4-demo \
  --dataset.single_task="Perform a bimanual manipulation task" \
  --dataset.fps=30 \
  --dataset.num_episodes=10 \
  --dataset.episode_time_s=300 \
  --dataset.reset_time_s=60 \
  --dataset.streaming_encoding=true \
  --dataset.vcodec=auto \
  --display_data=false \
  --resume=true
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

The Pico4 SDK Python module `xensevr_pc_service_sdk` must be installed in the
same environment as this plugin.
