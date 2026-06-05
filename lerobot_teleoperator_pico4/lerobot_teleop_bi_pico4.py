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
                "left_gripper.pos": 9,
                "right_tcp.x": 10,
                "right_tcp.y": 11,
                "right_tcp.z": 12,
                "right_tcp.r1": 13,
                "right_tcp.r2": 14,
                "right_tcp.r3": 15,
                "right_tcp.r4": 16,
                "right_tcp.r5": 17,
                "right_tcp.r6": 18,
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

    def get_action(self) -> RobotAction:
        if not self._is_connected or self._xrt is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        left_action = self._left_pico4.get_action()
        right_action = self._right_pico4.get_action()
        return {
            **self._prefix_action("left", left_action),
            **self._prefix_action("right", right_action),
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
