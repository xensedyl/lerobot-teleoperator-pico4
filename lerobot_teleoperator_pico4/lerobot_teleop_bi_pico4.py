import logging
import time
from typing import Any

import numpy as np

from lerobot.processor import RobotAction
from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from .config_bi_pico4 import BiPico4Config
from .config_pico4 import Pico4Config
from .lerobot_teleop_pico4 import Pico4


logger = logging.getLogger(__name__)

DEFAULT_TCP_POSE_QUAT = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float32)


def _normalize_quaternion(q: np.ndarray, input_format: str = "wxyz") -> np.ndarray:
    q = np.asarray(q, dtype=np.float32).reshape(-1)
    if len(q) != 4:
        raise ValueError(f"Quaternion must have 4 components, got {len(q)}.")

    norm = np.linalg.norm(q)
    if norm < 1e-10:
        if input_format == "xyzw":
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)

    q = q / norm
    if input_format == "wxyz":
        return q.astype(np.float32)
    if input_format == "xyzw":
        return np.array([q[3], q[0], q[1], q[2]], dtype=np.float32)
    raise ValueError(f"Unknown quaternion format {input_format!r}.")


def _quaternion_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    qw1, qx1, qy1, qz1 = q1
    qw2, qx2, qy2, qz2 = q2
    return np.array(
        [
            qw1 * qw2 - qx1 * qx2 - qy1 * qy2 - qz1 * qz2,
            qw1 * qx2 + qx1 * qw2 + qy1 * qz2 - qz1 * qy2,
            qw1 * qy2 - qx1 * qz2 + qy1 * qw2 + qz1 * qx2,
            qw1 * qz2 + qx1 * qy2 - qy1 * qx2 + qz1 * qw2,
        ],
        dtype=np.float32,
    )


def _quaternion_inverse(q: np.ndarray) -> np.ndarray:
    qw, qx, qy, qz = q
    norm_sq = qw * qw + qx * qx + qy * qy + qz * qz
    if norm_sq < 1e-10:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
    return np.array([qw, -qx, -qy, -qz], dtype=np.float32) / norm_sq


def _slerp_quaternion(q1: np.ndarray, q2: np.ndarray, t: float) -> np.ndarray:
    q1 = _normalize_quaternion(q1, input_format="wxyz")
    q2 = _normalize_quaternion(q2, input_format="wxyz")
    dot = float(np.dot(q1, q2))
    if dot < 0.0:
        q2 = -q2
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        return _normalize_quaternion(q1 + t * (q2 - q1), input_format="wxyz")

    theta = np.arccos(dot)
    sin_theta = np.sin(theta)
    w1 = np.sin((1.0 - t) * theta) / sin_theta
    w2 = np.sin(t * theta) / sin_theta
    return _normalize_quaternion(w1 * q1 + w2 * q2, input_format="wxyz")

def _quaternion_to_euler(qw: float, qx: float, qy: float, qz: float) -> np.ndarray:
    """Convert quaternion [qw, qx, qy, qz] to Euler angles (roll, pitch, yaw).

    Uses ZYX intrinsic rotation order, consistent with Flexiv SDK and aerospace standard.
    This is the inverse of euler_to_quaternion().

    Note: Gimbal lock occurs when pitch ≈ ±90°, causing roll and yaw to become coupled.

    Args:
        qw: Quaternion scalar component
        qx: Quaternion x component
        qy: Quaternion y component
        qz: Quaternion z component

    Returns:
        np.ndarray of shape (3,) in [roll, pitch, yaw] order (radians):
        - roll: Rotation around x-axis, range [-π, π]
        - pitch: Rotation around y-axis, range [-π/2, π/2]
        - yaw: Rotation around z-axis, range [-π, π]
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation) with gimbal lock handling
    sinp = np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0)
    pitch = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return np.array([roll, pitch, yaw], dtype=np.float32)




class BiPico4(Teleoperator):
    """Bimanual Pico4 teleoperator using both controllers through one SDK connection."""

    config_class = BiPico4Config
    name = "bi_pico4"

    def __init__(self, config: BiPico4Config):
        super().__init__(config)
        self.config = config
        self._is_connected = False
        self._xrt = None
        self._was_reset_button_pressed = False

        self._left_pico4 = Pico4(
            Pico4Config(
                id=f"{config.id}_left",
                use_left_controller=True,
                use_right_controller=False,
                pos_sensitivity=config.pos_sensitivity,
                ori_sensitivity=config.ori_sensitivity,
                filter_window_size=config.filter_window_size,
                gripper_width=config.left_gripper_width,
                invert_gripper=config.invert_gripper,
                grip_enable_threshold=config.grip_enable_threshold,
                grip_disable_threshold=config.grip_disable_threshold,
                orientation_offset_warning_deg=config.orientation_offset_warning_deg,
                target_tcp_drift_max_deg=config.target_tcp_drift_max_deg,
                position_jump_threshold=config.position_jump_threshold,
                max_pos_velocity=config.max_pos_velocity,
                max_rot_velocity=config.max_rot_velocity,
            )
        )
        self._right_pico4 = Pico4(
            Pico4Config(
                id=f"{config.id}_right",
                use_left_controller=False,
                use_right_controller=True,
                pos_sensitivity=config.pos_sensitivity,
                ori_sensitivity=config.ori_sensitivity,
                filter_window_size=config.filter_window_size,
                gripper_width=config.right_gripper_width,
                invert_gripper=config.invert_gripper,
                grip_enable_threshold=config.grip_enable_threshold,
                grip_disable_threshold=config.grip_disable_threshold,
                orientation_offset_warning_deg=config.orientation_offset_warning_deg,
                target_tcp_drift_max_deg=config.target_tcp_drift_max_deg,
                position_jump_threshold=config.position_jump_threshold,
                max_pos_velocity=config.max_pos_velocity,
                max_rot_velocity=config.max_rot_velocity,
            )
        )

    @property
    def requires_current_tcp_pose(self) -> bool:
        return True

    @property
    def action_features(self) -> dict[str, Any]:
        return {
            "dtype": "float32",
            "shape": (20,),
            # Order: left arm, right arm, left gripper, right gripper (matches the
            # TRON2 robot's action_features layout).
            "names": {
                "left_tcp.x": 0,
                "left_tcp.y": 1,
                "left_tcp.z": 2,
                "left_tcp.r1": 3,
                "left_tcp.r2": 4,
                "left_tcp.r3": 5,
                "left_tcp.r4": 6,
                "left_tcp.r5": 7,
                "left_tcp.r6": 8,
                "right_tcp.x": 9,
                "right_tcp.y": 10,
                "right_tcp.z": 11,
                "right_tcp.r1": 12,
                "right_tcp.r2": 13,
                "right_tcp.r3": 14,
                "right_tcp.r4": 15,
                "right_tcp.r5": 16,
                "right_tcp.r6": 17,
                "left_gripper.pos": 18,
                "right_gripper.pos": 19,
            },
        }

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        return self._is_connected

    def configure(self) -> None:
        pass

    def calibrate(self) -> None:
        pass

    def set_current_tcp_poses(self, left_tcp_pose_quat: np.ndarray, right_tcp_pose_quat: np.ndarray) -> None:
        self._left_pico4.set_current_tcp_pose(left_tcp_pose_quat)
        self._right_pico4.set_current_tcp_pose(right_tcp_pose_quat)

    def set_current_tcp_pose(self, current_tcp_pose_quat) -> None:
        left_pose, right_pose = current_tcp_pose_quat
        self.set_current_tcp_poses(left_pose, right_pose)

    def reset_to_current_tcp_pose(self) -> None:
        self._left_pico4.reset_to_current_tcp_pose()
        self._right_pico4.reset_to_current_tcp_pose()

    def _init_child(self, child: Pico4, xrt, tcp_pose_quat: np.ndarray) -> None:
        child._xrt = xrt
        child.set_current_tcp_pose(tcp_pose_quat)
        child._sync_target_to_current_tcp_pose()
        child._start_pos = child._target_pos.copy()
        child._start_quat = child._target_quat.copy()
        child._ref_pos = None
        child._quat_offset = None
        child._enabled = False
        child._was_enabled = False
        child._orientation_control_active = True
        child._last_raw_pose = None
        child._jump_filter_count = 0
        child._last_grip = 0.0
        child._last_a_button = False
        child._last_b_button = False
        child._last_x_button = False
        child._last_y_button = False
        child._was_reset_button_pressed = False
        child._last_action_time = None
        child._prev_target_pos = None
        child._prev_target_quat = None
        child._is_connected = True

    def connect(
        self,
        calibrate: bool = True,
        left_tcp_pose_quat: np.ndarray | None = None,
        right_tcp_pose_quat: np.ndarray | None = None,
    ) -> None:
        if self._is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        try:
            import xensevr_pc_service_sdk as xrt
        except ImportError as e:
            raise ImportError(
                "xensevr_pc_service_sdk is required for Pico4 teleoperation. "
                "Install the Pico4 PC service pybind package before running --teleop.type=bi_pico4."
            ) from e

        left_tcp_pose_quat = (
            DEFAULT_TCP_POSE_QUAT.copy()
            if left_tcp_pose_quat is None
            else np.asarray(left_tcp_pose_quat, dtype=np.float32)
        )
        right_tcp_pose_quat = (
            DEFAULT_TCP_POSE_QUAT.copy()
            if right_tcp_pose_quat is None
            else np.asarray(right_tcp_pose_quat, dtype=np.float32)
        )

        logger.info("Connecting to Pico4 VR headset with both controllers...")
        try:
            xrt.init()
            self._xrt = xrt
            time.sleep(0.5)

            for attempt in range(25):
                left_pose = xrt.get_left_controller_pose()
                right_pose = xrt.get_right_controller_pose()
                left_ok = any(abs(v) > 1e-6 for v in left_pose)
                right_ok = any(abs(v) > 1e-6 for v in right_pose)
                if left_ok and right_ok:
                    logger.info("Pico4 left/right controller data received on attempt %d.", attempt + 1)
                    break
                time.sleep(0.1)
            else:
                self._xrt = None
                raise DeviceNotConnectedError(
                    "Pico4 controller data is all zero. Restart the Pico4 VR client, "
                    "check the PC service, and make sure both controllers are paired."
                )

            self._init_child(self._left_pico4, xrt, left_tcp_pose_quat)
            self._init_child(self._right_pico4, xrt, right_tcp_pose_quat)
            self._is_connected = True
            self.poll_buttons()
            self._was_reset_button_pressed = self._right_pico4._last_a_button
            logger.info("%s connected.", self)
        except Exception:
            if self._xrt is not None:
                try:
                    self._xrt.close()
                except Exception:
                    logger.debug("Failed to close Pico4 SDK after connect error.", exc_info=True)
            self._xrt = None
            self._is_connected = False
            self._left_pico4._is_connected = False
            self._right_pico4._is_connected = False
            raise

    @staticmethod
    def _prefix_action(side: str, action: RobotAction) -> RobotAction:
        return {f"{side}_{key}": value for key, value in action.items()}

    def _filter_raw_pose(self, controller_pose_raw: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        pos = controller_pose_raw[:3].copy()
        quat = _normalize_quaternion(
            np.array(
                [
                    controller_pose_raw[6],
                    controller_pose_raw[3],
                    controller_pose_raw[4],
                    controller_pose_raw[5],
                ],
                dtype=np.float32,
            ),
            input_format="wxyz",
        )

        if self.config.filter_window_size <= 1:
            return pos, quat

        if self._raw_pos_queue.full():
            self._raw_pos_queue.get()
        self._raw_pos_queue.put(pos)
        filtered_pos = np.mean(np.array(list(self._raw_pos_queue.queue)), axis=0)

        if self._raw_quat_queue.full():
            self._raw_quat_queue.get()
        self._raw_quat_queue.put(quat)
        quat_list = list(self._raw_quat_queue.queue)
        filtered_quat = quat_list[0]
        for idx, next_quat in enumerate(quat_list[1:], start=1):
            filtered_quat = _slerp_quaternion(filtered_quat, next_quat, 1.0 / (idx + 1))

        return filtered_pos.astype(np.float32), filtered_quat

    def _transform_pico_to_robot_coordinate(
        self, pos: np.ndarray, quat: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        # Same convention as the existing Pico4 implementation: Pico X right,
        # Pico Y up, Pico Z toward user -> robot X forward, Y left, Z up.
        transformed_pos = np.array([-pos[2], -pos[0], pos[1]], dtype=np.float32)

        q_frame_transform = np.array([0.5, 0.5, -0.5, -0.5], dtype=np.float32)
        transformed_quat = _quaternion_multiply(
            _quaternion_multiply(q_frame_transform, quat),
            _quaternion_inverse(q_frame_transform),
        )
        return transformed_pos, _normalize_quaternion(transformed_quat, input_format="wxyz")

    def _read_headset_pose(self) -> np.ndarray:
        if self.config.use_headset:
            headset_pose = self._xrt.get_headset_pose()
            return np.asarray(headset_pose, dtype=np.float32)
        else:
            return np.array([0, 0, 0, 1, 0, 0, 0, 0], dtype=np.float32)

    def get_action(self) -> RobotAction:
        if not self._is_connected or self._xrt is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        left_action = self._prefix_action("left", self._left_pico4.get_action())
        right_action = self._prefix_action("right", self._right_pico4.get_action())

        headset_pose = self._read_headset_pose()
        filtered_pos_headset, filtered_quat_headset = self._filter_raw_pose(headset_pose)
        filtered_pos_headset_robot, filtered_quat_headset_robot = self._transform_pico_to_robot_coordinate(
            filtered_pos_headset,
            filtered_quat_headset,
        )
        rpy_robot = _quaternion_to_euler(filtered_quat_headset_robot[0], filtered_quat_headset_robot[1], filtered_quat_headset_robot[2], filtered_quat_headset_robot[3])

        # Emit in the action_features order: left arm, right arm, left gripper,
        # right gripper (grippers last, matching the TRON2 robot layout).
        left_gripper = left_action.pop("left_gripper.pos")
        right_gripper = right_action.pop("right_gripper.pos")

        if self.config.use_headset:

            return {
                **left_action,
                **right_action,
                "left_gripper.pos": left_gripper,
                "right_gripper.pos": right_gripper,
                "head.pitch": float(rpy_robot[1]),
                "head.yaw": float(rpy_robot[2]),
            }
        else:
            return {
                **left_action,
                **right_action,
                "left_gripper.pos": left_gripper,
                "right_gripper.pos": right_gripper,
            }

    def poll_buttons(self) -> None:
        if not self._is_connected or self._xrt is None:
            return
        self._right_pico4._last_a_button = bool(self._xrt.get_A_button())

    def get_reset_button(self) -> bool:
        current_pressed = self._right_pico4._last_a_button
        just_pressed = current_pressed and not self._was_reset_button_pressed
        self._was_reset_button_pressed = current_pressed
        return just_pressed

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        raise NotImplementedError("BiPico4 teleoperator does not support feedback.")

    def disconnect(self) -> None:
        if not self._is_connected or self._xrt is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        try:
            self._xrt.close()
        finally:
            self._xrt = None
            self._is_connected = False
            self._left_pico4._xrt = None
            self._right_pico4._xrt = None
            self._left_pico4._is_connected = False
            self._right_pico4._is_connected = False
        logger.info("%s disconnected.", self)
