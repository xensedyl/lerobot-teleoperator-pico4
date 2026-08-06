import builtins
import sys
import types

import pytest
from lerobot.teleoperators.config import TeleoperatorConfig
from lerobot.teleoperators.utils import make_teleoperator_from_config

from lerobot_teleoperator_pico4 import (
    BiPico4,
    BiPico4Config,
    BiPico4Head,
    BiPico4HeadConfig,
    Pico4,
    Pico4Config,
    Pico4Head,
    Pico4HeadConfig,
)
from lerobot_teleoperator_pico4.action_compatibility import (
    check_teleop_robot_action_compatibility,
)


def make_fake_xrt():
    fake_xrt = types.ModuleType("xensevr_pc_service_sdk")
    fake_xrt.init_calls = 0
    fake_xrt.close_calls = 0
    fake_xrt.init = lambda: setattr(fake_xrt, "init_calls", fake_xrt.init_calls + 1)
    fake_xrt.close = lambda: setattr(fake_xrt, "close_calls", fake_xrt.close_calls + 1)
    fake_xrt.get_left_controller_pose = lambda: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    fake_xrt.get_right_controller_pose = lambda: [-0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    fake_xrt.get_headset_pose = lambda: [0.0, 1.6, 0.0, 0.0, 0.0, 0.0, 1.0]
    fake_xrt.get_A_button = lambda: False
    fake_xrt.get_B_button = lambda: False
    fake_xrt.get_X_button = lambda: False
    fake_xrt.get_Y_button = lambda: False
    return fake_xrt


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


def test_bi_pico4_head_config_registered():
    cfg = BiPico4HeadConfig()

    assert cfg.type == "bi_pico4_head"
    assert TeleoperatorConfig.get_choice_class("bi_pico4_head") is BiPico4HeadConfig


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


def test_make_bi_pico4_head_has_composite_29d_schema(tmp_path):
    teleop = make_teleoperator_from_config(BiPico4HeadConfig(calibration_dir=tmp_path))

    assert isinstance(teleop, BiPico4Head)
    assert teleop.name == "bi_pico4_head"
    assert teleop.action_features["shape"] == (29,)
    assert list(teleop.action_features["names"])[18:23] == [
        "left_gripper.pos",
        "right_gripper.pos",
        "head_tcp.x",
        "head_tcp.y",
        "head_tcp.z",
    ]
    assert teleop.action_features["names"]["head_tcp.r6"] == 28

    pose = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0]
    teleop.set_current_tcp_pose((pose, pose, pose[:7]))
    assert teleop.requires_current_tcp_pose
    assert not teleop.left_enabled
    assert not teleop.right_enabled
    assert not teleop.head_enabled


def test_bi_pico4_head_action_compatibility_is_robot_vendor_neutral(tmp_path):
    teleop = BiPico4Head(BiPico4HeadConfig(calibration_dir=tmp_path))
    robot = type(
        "GenericBimanualHeadRobot",
        (),
        {
            "name": "generic_bimanual_head",
            "action_features": {key: float for key in teleop.action_features["names"]},
        },
    )()

    check_teleop_robot_action_compatibility(teleop, robot)


def test_bi_pico4_head_connects_controllers_and_headset_with_one_sdk_init(
    monkeypatch, tmp_path
):
    fake_xrt = make_fake_xrt()
    monkeypatch.setitem(sys.modules, "xensevr_pc_service_sdk", fake_xrt)

    teleop = BiPico4Head(BiPico4HeadConfig(calibration_dir=tmp_path))
    pose = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0]
    teleop.pre_init()
    teleop.connect(
        left_tcp_pose_quat=pose,
        right_tcp_pose_quat=pose,
        head_tcp_pose_quat=pose[:7],
    )

    assert fake_xrt.init_calls == 1
    assert teleop._left_pico4._xrt is fake_xrt
    assert teleop._right_pico4._xrt is fake_xrt
    assert teleop._head_pico4._xrt is fake_xrt

    teleop.disconnect()
    assert fake_xrt.close_calls == 1


@pytest.mark.parametrize(
    ("teleop_factory", "expected_readers"),
    [
        (lambda path: Pico4(Pico4Config(calibration_dir=path)), {"right controller"}),
        (lambda path: BiPico4(BiPico4Config(calibration_dir=path)), {"left controller", "right controller"}),
        (lambda path: Pico4Head(Pico4HeadConfig(calibration_dir=path)), {"headset"}),
        (
            lambda path: BiPico4Head(BiPico4HeadConfig(calibration_dir=path)),
            {"left controller", "right controller", "headset"},
        ),
    ],
)
def test_all_pico_teleoperators_reuse_pico4_pre_init(
    monkeypatch, tmp_path, teleop_factory, expected_readers
):
    fake_xrt = make_fake_xrt()
    monkeypatch.setitem(sys.modules, "xensevr_pc_service_sdk", fake_xrt)
    teleop = teleop_factory(tmp_path)

    assert set(teleop._sdk_pose_readers(fake_xrt)) == expected_readers
    teleop.pre_init()
    teleop.pre_init()

    assert fake_xrt.init_calls == 1
    assert teleop._xrt is fake_xrt
    teleop.connect()
    assert teleop.is_connected
    assert fake_xrt.init_calls == 1
    teleop.disconnect()
    assert fake_xrt.close_calls == 1


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
