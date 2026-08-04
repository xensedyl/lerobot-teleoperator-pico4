from lerobot.teleoperators.config import TeleoperatorConfig
from lerobot.teleoperators.utils import make_teleoperator_from_config

from lerobot_teleoperator_pico4 import (
    BiPico4,
    BiPico4Config,
    Pico4,
    Pico4Config,
    Pico4Head,
    Pico4HeadConfig,
)


def test_pico4_config_registered():
    cfg = Pico4Config()

    assert cfg.type == "pico4"
    assert TeleoperatorConfig.get_choice_class("pico4") is Pico4Config


def test_bi_pico4_config_registered():
    cfg = BiPico4Config()

    assert cfg.type == "bi_pico4"
    assert TeleoperatorConfig.get_choice_class("bi_pico4") is BiPico4Config


def test_pico4head_config_registered():
    cfg = Pico4HeadConfig()

    assert cfg.type == "pico4head"
    assert TeleoperatorConfig.get_choice_class("pico4head") is Pico4HeadConfig


def test_make_teleoperator_from_config_uses_plugin_class():
    teleop = make_teleoperator_from_config(Pico4Config())

    assert isinstance(teleop, Pico4)
    assert teleop.name == "pico4"


def test_make_bi_teleoperator_from_config_uses_plugin_class():
    teleop = make_teleoperator_from_config(BiPico4Config())

    assert isinstance(teleop, BiPico4)
    assert teleop.name == "bi_pico4"
    assert "left_tcp.x" in teleop.action_features["names"]
    assert "right_tcp.x" in teleop.action_features["names"]


def test_make_pico4head_from_config_uses_plugin_class(tmp_path):
    teleop = make_teleoperator_from_config(Pico4HeadConfig(calibration_dir=tmp_path))

    assert isinstance(teleop, Pico4Head)
    assert teleop.name == "pico4head"
    assert list(teleop.action_features["names"]) == [
        "tcp.x",
        "tcp.y",
        "tcp.z",
        "tcp.r1",
        "tcp.r2",
        "tcp.r3",
        "tcp.r4",
        "tcp.r5",
        "tcp.r6",
    ]
