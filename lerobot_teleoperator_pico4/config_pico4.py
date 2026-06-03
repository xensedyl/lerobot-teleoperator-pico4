from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("pico4")
@dataclass
class Pico4Config(TeleoperatorConfig):
    """Configuration for Pico4 controller teleoperation."""

    id: str = "pico4"
    use_left_controller: bool = False
    use_right_controller: bool = True
    pos_sensitivity: float = 1.0
    ori_sensitivity: float = 1.0
    filter_window_size: int = 1
    gripper_width: float = 1.0
    grip_enable_threshold: float = 0.5
    grip_disable_threshold: float = 0.3
    orientation_offset_warning_deg: float = 180.0
    target_tcp_drift_max_deg: float = 45.0
    position_jump_threshold: float = 0.1
    max_pos_velocity: float = 1.0
    max_rot_velocity: float = 6.28
