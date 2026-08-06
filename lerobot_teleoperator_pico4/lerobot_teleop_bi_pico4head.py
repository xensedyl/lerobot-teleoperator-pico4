import logging
from typing import Any

import numpy as np
from lerobot.processor import RobotAction

from .config_bi_pico4head import BiPico4HeadConfig
from .config_pico4head import Pico4HeadConfig
from .lerobot_teleop_bi_pico4 import BiPico4
from .lerobot_teleop_pico4head import Pico4Head

logger = logging.getLogger(__name__)

DEFAULT_HEAD_TCP_POSE_QUAT = np.array(
    [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0], dtype=np.float32
)


class BiPico4Head(BiPico4):
    """Controllers + headset teleoperator sharing one XenseVR SDK connection."""

    config_class = BiPico4HeadConfig
    name = "bi_pico4_head"

    def __init__(self, config: BiPico4HeadConfig):
        super().__init__(config)
        self.config = config
        self._head_pico4 = Pico4Head(
            Pico4HeadConfig(
                id=f"{config.id}_head",
                calibration_dir=config.calibration_dir,
                pos_sensitivity=config.head_pos_sensitivity,
                ori_sensitivity=config.head_ori_sensitivity,
                filter_window_size=config.head_filter_window_size,
                orientation_offset_warning_deg=config.head_orientation_offset_warning_deg,
                position_jump_threshold=config.head_position_jump_threshold,
                max_pos_velocity=config.head_max_pos_velocity,
                max_rot_velocity=config.head_max_rot_velocity,
            )
        )

    @property
    def left_enabled(self) -> bool:
        return bool(self._left_pico4._enabled)

    @property
    def right_enabled(self) -> bool:
        return bool(self._right_pico4._enabled)

    @property
    def head_enabled(self) -> bool:
        return bool(self._head_pico4._enabled)

    @property
    def action_features(self) -> dict[str, Any]:
        # Generic bimanual-plus-head layout: both 9D arm poses, both grippers,
        # then the 9D head pose.
        names = {
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
            "head_tcp.x": 20,
            "head_tcp.y": 21,
            "head_tcp.z": 22,
            "head_tcp.r1": 23,
            "head_tcp.r2": 24,
            "head_tcp.r3": 25,
            "head_tcp.r4": 26,
            "head_tcp.r5": 27,
            "head_tcp.r6": 28,
        }
        return {"dtype": "float32", "shape": (29,), "names": names}

    def set_current_tcp_poses(
        self,
        left_tcp_pose_quat: np.ndarray,
        right_tcp_pose_quat: np.ndarray,
        head_tcp_pose_quat: np.ndarray | None = None,
    ) -> None:
        super().set_current_tcp_poses(left_tcp_pose_quat, right_tcp_pose_quat)
        if head_tcp_pose_quat is not None:
            self._head_pico4.set_current_tcp_pose(head_tcp_pose_quat)

    def set_current_tcp_pose(self, current_tcp_pose_quat) -> None:
        try:
            left_pose, right_pose, head_pose = current_tcp_pose_quat
        except (TypeError, ValueError) as e:
            raise ValueError(
                "BiPico4Head expects (left_tcp_pose, right_tcp_pose, head_tcp_pose)."
            ) from e
        self.set_current_tcp_poses(left_pose, right_pose, head_pose)

    def set_current_head_tcp_pose(self, head_tcp_pose_quat: np.ndarray) -> None:
        self._head_pico4.set_current_tcp_pose(head_tcp_pose_quat)

    def reset_to_current_tcp_pose(self) -> None:
        super().reset_to_current_tcp_pose()
        self._head_pico4.reset_to_current_tcp_pose()

    def reset_to_pose(
        self,
        left_pose_7d: np.ndarray,
        right_pose_7d: np.ndarray,
        left_gripper_pos: float = 0.0,
        right_gripper_pos: float = 0.0,
        head_pose_7d: np.ndarray | None = None,
    ) -> None:
        left_pose = np.concatenate(
            [np.asarray(left_pose_7d, dtype=np.float32), [left_gripper_pos]]
        ).astype(np.float32)
        right_pose = np.concatenate(
            [np.asarray(right_pose_7d, dtype=np.float32), [right_gripper_pos]]
        ).astype(np.float32)
        self.set_current_tcp_poses(left_pose, right_pose, head_pose_7d)
        self.reset_to_current_tcp_pose()

    def _sdk_pose_readers(self, xrt):
        readers = super()._sdk_pose_readers(xrt)
        readers["headset"] = xrt.get_headset_pose
        return readers

    def _init_head_child(self, xrt, head_tcp_pose_quat: np.ndarray) -> None:
        head = self._head_pico4
        head._xrt = xrt
        head.set_current_tcp_pose(head_tcp_pose_quat)
        head._sync_target_to_current_tcp_pose()
        head._start_pos = head._target_pos.copy()
        head._start_quat = head._target_quat.copy()
        head._ref_pos = None
        head._quat_offset = None
        head._enabled = False
        head._was_enabled = False
        head._last_x_button = False
        head._last_a_button = False
        head._was_reset_button_pressed = False
        head._orientation_control_active = True
        head._last_raw_pose = None
        head._jump_filter_count = 0
        head._last_action_time = None
        head._prev_target_pos = None
        head._prev_target_quat = None
        head._is_connected = True

    def connect(
        self,
        calibrate: bool = True,
        left_tcp_pose_quat: np.ndarray | None = None,
        right_tcp_pose_quat: np.ndarray | None = None,
        head_tcp_pose_quat: np.ndarray | None = None,
    ) -> None:
        head_tcp_pose_quat = (
            DEFAULT_HEAD_TCP_POSE_QUAT.copy()
            if head_tcp_pose_quat is None
            else np.asarray(head_tcp_pose_quat, dtype=np.float32)
        )
        super().connect(
            calibrate=calibrate,
            left_tcp_pose_quat=left_tcp_pose_quat,
            right_tcp_pose_quat=right_tcp_pose_quat,
        )
        try:
            self._init_head_child(self._xrt, head_tcp_pose_quat)
        except Exception:
            super().disconnect()
            raise
        logger.info("%s connected with both controllers and headset.", self)

    def get_action(self) -> RobotAction:
        arms_action = super().get_action()
        head_action = self._head_pico4.get_action()
        return {
            **arms_action,
            **{f"head_{key}": value for key, value in head_action.items()},
        }

    def disconnect(self) -> None:
        self._head_pico4._xrt = None
        self._head_pico4._is_connected = False
        super().disconnect()
