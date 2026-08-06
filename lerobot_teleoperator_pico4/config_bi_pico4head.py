from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig

from .config_bi_pico4 import BiPico4Config


@TeleoperatorConfig.register_subclass("bi_pico4_head")
@dataclass
class BiPico4HeadConfig(BiPico4Config):
    """Both Pico controllers for the arms plus the headset for an active head."""

    id: str = "bi_pico4_head"

    head_pos_sensitivity: float = 0.5
    head_ori_sensitivity: float = 0.5
    head_filter_window_size: int = 3
    head_orientation_offset_warning_deg: float = 180.0
    head_position_jump_threshold: float = 0.1
    head_max_pos_velocity: float = 0.25
    head_max_rot_velocity: float = 1.0
    # Head control is fixed to the left-controller X button and tracks both
    # headset position and orientation.

    def __post_init__(self) -> None:
        if self.head_filter_window_size < 1:
            raise ValueError("head_filter_window_size must be >= 1.")
        if self.head_pos_sensitivity < 0 or self.head_ori_sensitivity < 0:
            raise ValueError("Head sensitivities must be >= 0.")
        if self.head_max_pos_velocity < 0 or self.head_max_rot_velocity < 0:
            raise ValueError("Head velocity limits must be >= 0.")
