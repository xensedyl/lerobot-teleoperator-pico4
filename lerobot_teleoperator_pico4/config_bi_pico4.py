from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("bi_pico4")
@dataclass
class BiPico4Config(TeleoperatorConfig):
    """Configuration for bimanual Pico4 controller teleoperation."""

    id: str = "bi_pico4"

    pos_sensitivity: float = 1.0
    ori_sensitivity: float = 1.0
    filter_window_size: int = 1

    left_gripper_width: float = 1.0
    right_gripper_width: float = 1.0

    grip_enable_threshold: float = 0.5
    grip_disable_threshold: float = 0.3
    orientation_offset_warning_deg: float = 180.0
    target_tcp_drift_max_deg: float = 45.0
    position_jump_threshold: float = 0.1
    max_pos_velocity: float = 1.0
    max_rot_velocity: float = 6.28
