import logging
import time
from queue import Queue
from typing import Any

import numpy as np
from lerobot.processor import RobotAction
from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from .config_pico4head import Pico4HeadConfig
from .lerobot_teleop_pico4 import (
    Pico4,
    _normalize_quaternion,
    _quaternion_inverse,
    _quaternion_multiply,
    _quaternion_to_rotation_6d,
    _slerp_quaternion,
)

logger = logging.getLogger(__name__)

DEFAULT_TCP_POSE_QUAT = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0], dtype=np.float32)


class Pico4Head(Pico4):
    """Pico4 headset teleoperator.

    The headset pose is applied relative to the robot's current TCP pose. The
    output action is tcp.x/y/z and tcp.r1-r6 using the 6D rotation representation.
    """

    config_class = Pico4HeadConfig
    name = "pico4head"

    def __init__(self, config: Pico4HeadConfig):
        # Reuse Pico4's SDK lifecycle without constructing controller state.
        Teleoperator.__init__(self, config)
        self.config = config
        self._is_connected = False
        self._xrt = None

        self._target_pos = np.zeros(3, dtype=np.float32)
        self._target_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self._current_tcp_pose_quat = DEFAULT_TCP_POSE_QUAT.copy()
        self._start_pos = np.zeros(3, dtype=np.float32)
        self._start_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self._ref_pos: np.ndarray | None = None
        self._quat_offset: np.ndarray | None = None

        self._raw_pos_queue: Queue = Queue(max(1, int(config.filter_window_size)))
        self._raw_quat_queue: Queue = Queue(max(1, int(config.filter_window_size)))

        self._enabled = False
        self._was_enabled = False
        self._last_x_button = False
        self._last_a_button = False
        self._was_reset_button_pressed = False
        self._orientation_control_active = True
        self._last_raw_pose: np.ndarray | None = None
        self._jump_filter_count = 0
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
            "shape": (9,),
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

    def _sdk_pose_readers(self, xrt):
        return {"headset": xrt.get_headset_pose}

    def connect(self, calibrate: bool = True) -> None:
        if self._is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        logger.info("Connecting to Pico4 VR headset...")
        try:
            self._get_preinitialized_xrt()

            self._sync_target_to_current_tcp_pose()
            self._start_pos = self._target_pos.copy()
            self._start_quat = self._target_quat.copy()
            self._ref_pos = None
            self._quat_offset = None
            self._enabled = False
            self._was_enabled = False
            self._last_x_button = False
            self._last_a_button = False
            self._was_reset_button_pressed = False
            self._orientation_control_active = True
            self._is_connected = True
            logger.info("%s connected.", self)
        except Exception:
            if self._xrt is not None:
                try:
                    self._close_sdk()
                except Exception:
                    logger.debug(
                        "Failed to close Pico4 SDK after connect error.", exc_info=True
                    )
            self._is_connected = False
            raise

    def configure(self) -> None:
        pass

    def calibrate(self) -> None:
        pass

    def set_current_tcp_pose(self, current_tcp_pose_quat: np.ndarray) -> None:
        current_tcp_pose_quat = np.asarray(current_tcp_pose_quat, dtype=np.float32)
        if current_tcp_pose_quat.shape not in {(7,), (8,)}:
            raise ValueError(
                "Pico4Head current TCP pose must be "
                "[x, y, z, qw, qx, qy, qz] with an optional gripper value."
            )
        self._current_tcp_pose_quat = current_tcp_pose_quat[:7].copy()

    def _sync_target_to_current_tcp_pose(self) -> None:
        self._target_pos = self._current_tcp_pose_quat[:3].copy()
        self._target_quat = _normalize_quaternion(
            self._current_tcp_pose_quat[3:7], input_format="wxyz"
        )

    def reset_to_current_tcp_pose(self) -> None:
        self._sync_target_to_current_tcp_pose()
        self._start_pos = self._target_pos.copy()
        self._start_quat = self._target_quat.copy()
        self._ref_pos = None
        self._quat_offset = None
        self._enabled = False
        self._was_enabled = False
        self._last_x_button = False
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

    def _read_headset_pose(self) -> np.ndarray:
        pose = np.asarray(self._xrt.get_headset_pose(), dtype=np.float32)
        if pose.size < 7:
            raise ValueError(
                "Pico4 headset pose must contain [x, y, z, qx, qy, qz, qw]."
            )
        return pose

    def _read_headset_state(self) -> tuple[np.ndarray, bool]:
        pose = self._read_headset_pose()
        self._last_x_button = bool(self._xrt.get_X_button())
        self._last_a_button = bool(self._xrt.get_A_button())
        return pose, self._last_x_button

    def get_reset_button(self) -> bool:
        """Return True once per A-button press to reset the head to its initial pose.

        The A (right primary) button is separate from the X button that hold-enables
        teleoperation, so resetting never interferes with motion control. Edge-detected so
        holding A triggers a single reset instead of repeating every loop iteration.
        """
        current_pressed = self._last_a_button
        just_pressed = current_pressed and not self._was_reset_button_pressed
        self._was_reset_button_pressed = current_pressed
        return just_pressed

    def _clear_pose_reference(self) -> None:
        self._ref_pos = None
        self._quat_offset = None
        self._last_raw_pose = None
        self._last_action_time = None
        self._prev_target_pos = None
        self._prev_target_quat = None

        while not self._raw_pos_queue.empty():
            self._raw_pos_queue.get()
        while not self._raw_quat_queue.empty():
            self._raw_quat_queue.get()

    def _filter_raw_pose(
        self, headset_pose_raw: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        pos = headset_pose_raw[:3].copy()
        quat = _normalize_quaternion(
            np.array(
                [
                    headset_pose_raw[6],
                    headset_pose_raw[3],
                    headset_pose_raw[4],
                    headset_pose_raw[5],
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

    @staticmethod
    def _transform_pico_to_robot_coordinate(
        pos: np.ndarray, quat: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        # Pico X right, Y up, Z toward user -> robot X forward, Y left, Z up.
        transformed_pos = np.array([-pos[2], -pos[0], pos[1]], dtype=np.float32)
        q_frame_transform = np.array([0.5, 0.5, -0.5, -0.5], dtype=np.float32)
        transformed_quat = _quaternion_multiply(
            _quaternion_multiply(q_frame_transform, quat),
            _quaternion_inverse(q_frame_transform),
        )
        return transformed_pos, _normalize_quaternion(
            transformed_quat, input_format="wxyz"
        )

    def _reset_reference(self, pos: np.ndarray, quat: np.ndarray) -> None:
        self._ref_pos = pos.copy()
        self._start_pos = self._target_pos.copy()
        self._start_quat = self._target_quat.copy()
        self._quat_offset = _quaternion_multiply(
            _quaternion_inverse(quat), self._target_quat
        )
        self._quat_offset = _normalize_quaternion(
            self._quat_offset, input_format="wxyz"
        )

        offset_angle_deg = float(
            np.degrees(2.0 * np.arccos(np.clip(abs(self._quat_offset[0]), 0.0, 1.0)))
        )
        self._orientation_control_active = (
            offset_angle_deg <= self.config.orientation_offset_warning_deg
        )
        if not self._orientation_control_active:
            logger.warning(
                "Pico4Head orientation offset %.1f deg exceeds threshold %.1f deg; "
                "orientation control disabled.",
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
                        self._target_pos = self._prev_target_pos + delta_pos * (
                            max_delta / delta_norm
                        )

                if (
                    self.config.max_rot_velocity > 0
                    and self._prev_target_quat is not None
                ):
                    dot = float(
                        np.clip(
                            abs(np.dot(self._target_quat, self._prev_target_quat)),
                            0.0,
                            1.0,
                        )
                    )
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

        headset_pose_raw, x_button_pressed = self._read_headset_state()
        was_enabled = self._enabled
        self._enabled = x_button_pressed
        just_enabled = self._enabled and not was_enabled
        just_disabled = was_enabled and not self._enabled

        if just_enabled:
            # Start from the robot's latest observed TCP and use the current
            # headset pose as a new reference, so re-enabling cannot jump.
            self._sync_target_to_current_tcp_pose()
            self._start_pos = self._target_pos.copy()
            self._start_quat = self._target_quat.copy()
            self._clear_pose_reference()
            logger.info("Pico4Head teleoperation enabled while X is held.")
        elif just_disabled:
            # Keep sending the last target while X is released. The next press
            # establishes a fresh headset reference before motion resumes.
            self._clear_pose_reference()
            logger.info("Pico4Head teleoperation disabled because X was released.")

        self._was_enabled = self._enabled

        if not self._enabled:
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
            }

        if self._last_raw_pose is not None and self.config.position_jump_threshold > 0:
            pos_delta = float(
                np.linalg.norm(headset_pose_raw[:3] - self._last_raw_pose[:3])
            )
            if pos_delta > self.config.position_jump_threshold:
                self._jump_filter_count += 1
                logger.warning(
                    "Pico4 headset position jump #%d: %.4fm > %.4fm; "
                    "clamping this frame.",
                    self._jump_filter_count,
                    pos_delta,
                    self.config.position_jump_threshold,
                )
                headset_pose_raw[:3] = self._last_raw_pose[:3]
                self._last_raw_pose = None
            else:
                self._last_raw_pose = headset_pose_raw.copy()
        else:
            self._last_raw_pose = headset_pose_raw.copy()

        filtered_pos_pico, filtered_quat_pico = self._filter_raw_pose(headset_pose_raw)
        filtered_pos_robot, filtered_quat_robot = (
            self._transform_pico_to_robot_coordinate(
                filtered_pos_pico, filtered_quat_pico
            )
        )

        if just_enabled or self._ref_pos is None:
            self._last_raw_pose = None
            self._reset_reference(filtered_pos_robot, filtered_quat_robot)

        rel_pos = filtered_pos_robot - self._ref_pos
        self._target_pos = self._start_pos + rel_pos * float(
            self.config.pos_sensitivity
        )

        if self._orientation_control_active and self._quat_offset is not None:
            full_target_quat = _quaternion_multiply(
                filtered_quat_robot, self._quat_offset
            )
            full_target_quat = _normalize_quaternion(
                full_target_quat, input_format="wxyz"
            )
            if self.config.ori_sensitivity < 1.0:
                self._target_quat = _slerp_quaternion(
                    self._start_quat,
                    full_target_quat,
                    float(self.config.ori_sensitivity),
                )
            else:
                self._target_quat = full_target_quat

        if (
            self._prev_target_quat is not None
            and np.dot(self._target_quat, self._prev_target_quat) < 0
        ):
            self._target_quat = -self._target_quat
        self._apply_rate_limit()

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
        }

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        raise NotImplementedError("Pico4Head teleoperator does not support feedback.")

    def disconnect(self) -> None:
        if not self._is_connected or self._xrt is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        try:
            self._close_sdk()
        finally:
            self._is_connected = False
        logger.info("%s disconnected.", self)
