import logging
import time
from queue import Queue
from typing import Any

import numpy as np

from lerobot.processor import RobotAction
from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from .config_pico4 import Pico4Config

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


def _quaternion_to_rotation_6d(q: np.ndarray) -> np.ndarray:
    qw, qx, qy, qz = _normalize_quaternion(q, input_format="wxyz")
    return np.array(
        [
            1.0 - 2.0 * (qy * qy + qz * qz),
            2.0 * (qx * qy + qz * qw),
            2.0 * (qx * qz - qy * qw),
            2.0 * (qx * qy - qz * qw),
            1.0 - 2.0 * (qx * qx + qz * qz),
            2.0 * (qy * qz + qx * qw),
        ],
        dtype=np.float32,
    )


class Pico4(Teleoperator):
    """Pico4 VR controller teleoperator.

    The output action is a Cartesian target:
    tcp.x/y/z, tcp.r1-r6 (6D rotation), and gripper.pos in [0, 1].
    """

    config_class = Pico4Config
    name = "pico4"

    def __init__(self, config: Pico4Config):
        super().__init__(config)
        self.config = config
        self._is_connected = False
        self._xrt = None

        self._target_pos = np.zeros(3, dtype=np.float32)
        self._target_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self._target_gripper_pos = 1.0
        self._current_tcp_pose_quat = DEFAULT_TCP_POSE_QUAT.copy()
        self._start_pos = np.zeros(3, dtype=np.float32)
        self._start_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self._ref_pos: np.ndarray | None = None
        self._quat_offset: np.ndarray | None = None

        self._raw_pos_queue: Queue = Queue(max(1, int(self.config.filter_window_size)))
        self._raw_quat_queue: Queue = Queue(max(1, int(self.config.filter_window_size)))

        self._enabled = False
        self._was_enabled = False
        self._orientation_control_active = True
        self._last_raw_pose: np.ndarray | None = None
        self._jump_filter_count = 0
        self._last_grip = 0.0
        self._last_a_button = False
        self._last_b_button = False
        self._last_x_button = False
        self._last_y_button = False
        self._was_reset_button_pressed = False

        self._last_action_time: float | None = None
        self._prev_target_pos: np.ndarray | None = None
        self._prev_target_quat: np.ndarray | None = None

    @property
    def requires_current_tcp_pose(self) -> bool:
        return True

    @property
    def action_features(self) -> dict[str, Any]:
        return {
            "dtype": "float32",
            "shape": (10,),
            "names": {
                "tcp.x": 0,
                "tcp.y": 1,
                "tcp.z": 2,
                "tcp.r1": 3,
                "tcp.r2": 4,
                "tcp.r3": 5,
                "tcp.r4": 6,
                "tcp.r5": 7,
                "tcp.r6": 8,
                "gripper.pos": 9,
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

    def connect(self, calibrate: bool = True) -> None:
        if self._is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        try:
            import xensevr_pc_service_sdk as xrt
        except ImportError as e:
            raise ImportError(
                "xensevr_pc_service_sdk is required for Pico4 teleoperation. "
                "Install the Pico4 PC service pybind package before running --teleop.type=pico4."
            ) from e

        logger.info("Connecting to Pico4 VR headset...")
        try:
            xrt.init()
            self._xrt = xrt
            time.sleep(0.5)

            for attempt in range(25):
                pose = self._read_controller_pose()
                if any(abs(v) > 1e-6 for v in pose):
                    logger.info("Pico4 controller data received on attempt %d.", attempt + 1)
                    break
                time.sleep(0.1)
            else:
                self._xrt = None
                raise DeviceNotConnectedError(
                    "Pico4 controller data is all zero. Restart the Pico4 VR client, "
                    "check the PC service, and make sure the selected controller is paired."
                )

            self._sync_target_to_current_tcp_pose()
            self._start_pos = self._target_pos.copy()

            self._ref_pos = None
            self._quat_offset = None
            self._enabled = False
            self._was_enabled = False
            self._orientation_control_active = True
            self._was_reset_button_pressed = False

            self._is_connected = True
            logger.info("%s connected.", self)
        except Exception:
            if self._xrt is not None:
                try:
                    self._xrt.close()
                except Exception:
                    logger.debug("Failed to close Pico4 SDK after connect error.", exc_info=True)
            self._xrt = None
            self._is_connected = False
            raise

    def configure(self) -> None:
        pass

    def calibrate(self) -> None:
        pass

    def set_current_tcp_pose(self, current_tcp_pose_quat: np.ndarray) -> None:
        current_tcp_pose_quat = np.asarray(current_tcp_pose_quat, dtype=np.float32)
        if current_tcp_pose_quat.shape != (8,):
            raise ValueError(
                "Pico4 current TCP pose must be [x, y, z, qw, qx, qy, qz, gripper_norm]."
            )
        self._current_tcp_pose_quat = current_tcp_pose_quat.copy()

    def _sync_target_to_current_tcp_pose(self) -> None:
        self._target_pos = self._current_tcp_pose_quat[:3].copy()
        self._target_quat = _normalize_quaternion(self._current_tcp_pose_quat[3:7], input_format="wxyz")
        self._target_gripper_pos = float(self._current_tcp_pose_quat[7])

    def reset_to_current_tcp_pose(self) -> None:
        self._sync_target_to_current_tcp_pose()
        self._start_pos = self._target_pos.copy()
        self._start_quat = self._target_quat.copy()
        self._ref_pos = None
        self._quat_offset = None
        self._enabled = False
        self._was_enabled = False
        self._orientation_control_active = True
        self._last_raw_pose = None
        self._jump_filter_count = 0
        self._last_action_time = None
        self._prev_target_pos = None
        self._prev_target_quat = None

        while not self._raw_pos_queue.empty():
            self._raw_pos_queue.get()
        while not self._raw_quat_queue.empty():
            self._raw_quat_queue.get()

    def _read_controller_pose(self):
        if self.config.use_right_controller:
            return self._xrt.get_right_controller_pose()
        if self.config.use_left_controller:
            return self._xrt.get_left_controller_pose()
        raise RuntimeError("No Pico4 controller enabled.")

    def _read_controller_state(self) -> tuple[np.ndarray, float, float]:
        if self.config.use_right_controller:
            pose = self._xrt.get_right_controller_pose()
            grip = float(self._xrt.get_right_grip())
            trigger = float(self._xrt.get_right_trigger())
            self._last_a_button = bool(self._xrt.get_A_button())
            self._last_b_button = bool(self._xrt.get_B_button())
        elif self.config.use_left_controller:
            pose = self._xrt.get_left_controller_pose()
            grip = float(self._xrt.get_left_grip())
            trigger = float(self._xrt.get_left_trigger())
            self._last_x_button = bool(self._xrt.get_X_button())
            self._last_y_button = bool(self._xrt.get_Y_button())
        else:
            raise RuntimeError("No Pico4 controller enabled.")

        return np.asarray(pose, dtype=np.float32), grip, trigger

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

    def _reset_reference(self, pos: np.ndarray, quat: np.ndarray) -> None:
        self._ref_pos = pos.copy()
        self._start_pos = self._target_pos.copy()
        self._start_quat = self._target_quat.copy()

        self._quat_offset = _quaternion_multiply(_quaternion_inverse(quat), self._target_quat)
        self._quat_offset = _normalize_quaternion(self._quat_offset, input_format="wxyz")

        offset_angle_deg = float(
            np.degrees(2.0 * np.arccos(np.clip(abs(self._quat_offset[0]), 0.0, 1.0)))
        )
        self._orientation_control_active = offset_angle_deg <= self.config.orientation_offset_warning_deg
        if not self._orientation_control_active:
            logger.warning(
                "Pico4 orientation offset %.1f deg exceeds threshold %.1f deg; orientation control disabled.",
                offset_angle_deg,
                self.config.orientation_offset_warning_deg,
            )

    def _apply_rate_limit(self) -> None:
        now = time.time()
        if self._prev_target_pos is not None and self._last_action_time is not None:
            dt = now - self._last_action_time
            if dt > 0:
                if self.config.max_pos_velocity > 0:
                    max_delta = float(self.config.max_pos_velocity) * dt
                    delta_pos = self._target_pos - self._prev_target_pos
                    delta_norm = float(np.linalg.norm(delta_pos))
                    if delta_norm > max_delta > 0:
                        self._target_pos = self._prev_target_pos + delta_pos * (max_delta / delta_norm)

                if self.config.max_rot_velocity > 0 and self._prev_target_quat is not None:
                    dot = float(np.clip(abs(np.dot(self._target_quat, self._prev_target_quat)), 0.0, 1.0))
                    angle = float(2.0 * np.arccos(dot))
                    max_angle = float(self.config.max_rot_velocity) * dt
                    if angle > max_angle > 0:
                        self._target_quat = _slerp_quaternion(
                            self._prev_target_quat,
                            self._target_quat,
                            max_angle / angle,
                        )

        self._prev_target_pos = self._target_pos.copy()
        self._prev_target_quat = self._target_quat.copy()
        self._last_action_time = now

    def get_action(self) -> RobotAction:
        if not self._is_connected or self._xrt is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        controller_pose_raw, controller_grip, controller_trigger = self._read_controller_state()
        self._last_grip = controller_grip

        if self._last_raw_pose is not None and self.config.position_jump_threshold > 0:
            pos_delta = float(np.linalg.norm(controller_pose_raw[:3] - self._last_raw_pose[:3]))
            if pos_delta > self.config.position_jump_threshold:
                self._jump_filter_count += 1
                logger.warning(
                    "Pico4 position jump #%d: %.4fm > %.4fm; clamping this frame.",
                    self._jump_filter_count,
                    pos_delta,
                    self.config.position_jump_threshold,
                )
                controller_pose_raw[:3] = self._last_raw_pose[:3]
                self._last_raw_pose = None
            else:
                self._last_raw_pose = controller_pose_raw.copy()
        else:
            self._last_raw_pose = controller_pose_raw.copy()

        was_enabled = self._enabled
        if self._enabled:
            self._enabled = controller_grip > self.config.grip_disable_threshold
        else:
            self._enabled = controller_grip > self.config.grip_enable_threshold
        just_enabled = self._enabled and not was_enabled

        filtered_pos_pico, filtered_quat_pico = self._filter_raw_pose(controller_pose_raw)
        filtered_pos_robot, filtered_quat_robot = self._transform_pico_to_robot_coordinate(
            filtered_pos_pico,
            filtered_quat_pico,
        )

        if just_enabled:
            actual_tcp_quat = _normalize_quaternion(self._current_tcp_pose_quat[3:7], input_format="wxyz")
            drift_dot = abs(float(np.dot(self._target_quat, actual_tcp_quat)))
            drift_angle_deg = float(np.degrees(2.0 * np.arccos(np.clip(drift_dot, 0.0, 1.0))))
            if drift_angle_deg > self.config.target_tcp_drift_max_deg:
                logger.warning(
                    "Pico4 target quaternion drift %.1f deg exceeds %.1f deg; resync and re-grip.",
                    drift_angle_deg,
                    self.config.target_tcp_drift_max_deg,
                )
                self._target_pos = self._current_tcp_pose_quat[:3].copy()
                self._target_quat = actual_tcp_quat
                self._enabled = False
                self._was_enabled = False
                self._prev_target_pos = None
                self._prev_target_quat = None
                self._last_action_time = None

        if just_enabled or self._ref_pos is None:
            self._last_raw_pose = None
            self._reset_reference(filtered_pos_robot, filtered_quat_robot)

        if self._enabled:
            rel_pos = filtered_pos_robot - self._ref_pos
            self._target_pos = self._start_pos + rel_pos * float(self.config.pos_sensitivity)

            if self._orientation_control_active and self._quat_offset is not None:
                full_target_quat = _quaternion_multiply(filtered_quat_robot, self._quat_offset)
                full_target_quat = _normalize_quaternion(full_target_quat, input_format="wxyz")
                if self.config.ori_sensitivity < 1.0:
                    self._target_quat = _slerp_quaternion(
                        self._start_quat,
                        full_target_quat,
                        float(self.config.ori_sensitivity),
                    )
                else:
                    self._target_quat = full_target_quat

        self._was_enabled = self._enabled

        if self._prev_target_quat is not None and np.dot(self._target_quat, self._prev_target_quat) < 0:
            self._target_quat = -self._target_quat
        self._apply_rate_limit()

        # B601 mapping: trigger released -> open, trigger pressed -> closed.
        self._target_gripper_pos = float(controller_trigger) * float(self.config.gripper_width)
        self._target_gripper_pos = float(np.clip(self._target_gripper_pos, 0.0, self.config.gripper_width))

        r6d = _quaternion_to_rotation_6d(self._target_quat)
        return {
            "tcp.x": float(self._target_pos[0]),
            "tcp.y": float(self._target_pos[1]),
            "tcp.z": float(self._target_pos[2]),
            "tcp.r1": float(r6d[0]),
            "tcp.r2": float(r6d[1]),
            "tcp.r3": float(r6d[2]),
            "tcp.r4": float(r6d[3]),
            "tcp.r5": float(r6d[4]),
            "tcp.r6": float(r6d[5]),
            "gripper.pos": float(self._target_gripper_pos),
        }

    def poll_buttons(self) -> None:
        if not self._is_connected or self._xrt is None:
            return
        if self.config.use_right_controller:
            self._last_a_button = bool(self._xrt.get_A_button())
            self._last_b_button = bool(self._xrt.get_B_button())
        if self.config.use_left_controller:
            self._last_x_button = bool(self._xrt.get_X_button())
            self._last_y_button = bool(self._xrt.get_Y_button())

    def get_reset_button(self) -> bool:
        current_pressed = self._last_a_button if self.config.use_right_controller else self._last_x_button
        just_pressed = current_pressed and not self._was_reset_button_pressed
        self._was_reset_button_pressed = current_pressed
        return just_pressed

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        raise NotImplementedError("Pico4 teleoperator does not support feedback.")

    def disconnect(self) -> None:
        if not self._is_connected or self._xrt is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        try:
            self._xrt.close()
        finally:
            self._xrt = None
            self._is_connected = False
        logger.info("%s disconnected.", self)
