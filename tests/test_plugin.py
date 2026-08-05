import builtins

import pytest

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
from lerobot_teleoperator_pico4.action_compatibility import (
    check_teleop_robot_action_compatibility,
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


def test_action_compatibility_uses_features_not_robot_type(tmp_path):
    teleop = Pico4Head(Pico4HeadConfig(calibration_dir=tmp_path))
    robot = type(
        "ArbitraryCartesianRobot",
        (),
        {
            "name": "a_robot_name_not_in_any_whitelist",
            "action_features": {
                "tcp.x": float,
                "tcp.y": float,
                "tcp.z": float,
                "tcp.r1": float,
                "tcp.r2": float,
                "tcp.r3": float,
                "tcp.r4": float,
                "tcp.r5": float,
                "tcp.r6": float,
            },
        },
    )()

    check_teleop_robot_action_compatibility(teleop, robot)


def test_action_compatibility_reports_missing_and_extra_fields(tmp_path):
    teleop = Pico4Head(Pico4HeadConfig(calibration_dir=tmp_path))
    robot = type(
        "IncompatibleRobot",
        (),
        {
            "name": "incompatible_robot",
            "action_features": {"tcp.x": float, "gripper.pos": float},
        },
    )()

    try:
        check_teleop_robot_action_compatibility(teleop, robot)
    except ValueError as error:
        message = str(error)
        assert "robot is missing" in message
        assert "tcp.r1" in message
        assert "robot has extra fields ['gripper.pos']" in message
    else:
        raise AssertionError(
            "Expected incompatible action features to raise ValueError."
        )


def test_pico4_connect_missing_sdk_prints_install_commands(monkeypatch):
    original_import = builtins.__import__

    def import_without_pico_sdk(name, *args, **kwargs):
        if name == "xensevr_pc_service_sdk":
            raise ImportError(name)
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", import_without_pico_sdk)
    teleop = Pico4(Pico4Config())

    with pytest.raises(ImportError) as exc_info:
        teleop.connect()

    message = str(exc_info.value)
    assert "bash setup_env.sh --install" in message
    assert "Xense-Pico-Teleop-Interface.git" in message
