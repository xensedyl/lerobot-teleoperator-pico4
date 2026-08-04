from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("pico4head")
@dataclass
class Pico4HeadConfig(TeleoperatorConfig):
    """Configuration for controlling a TCP target with the Pico4 headset pose."""

    id: str = "pico4head"
    pos_sensitivity: float = 1.0
    ori_sensitivity: float = 1.0
    filter_window_size: int = 1
    orientation_offset_warning_deg: float = 180.0
    position_jump_threshold: float = 0.1
    max_pos_velocity: float = 1.0
    max_rot_velocity: float = 6.28
