from lerobot.teleoperators.config import TeleoperatorConfig
from lerobot.teleoperators.utils import make_teleoperator_from_config

from lerobot_teleoperator_pico4 import BiPico4, BiPico4Config, Pico4, Pico4Config


def test_pico4_config_registered():
    cfg = Pico4Config()

    assert cfg.type == "pico4"
    assert TeleoperatorConfig.get_choice_class("pico4") is Pico4Config


def test_bi_pico4_config_registered():
    cfg = BiPico4Config()

    assert cfg.type == "bi_pico4"
    assert TeleoperatorConfig.get_choice_class("bi_pico4") is BiPico4Config


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
