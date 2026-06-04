import logging
import time
import traceback
from dataclasses import asdict, dataclass
from pprint import pformat

import rerun as rr

from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig  # noqa: F401
from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig  # noqa: F401
from lerobot.configs import parser
from lerobot.processor import (
    RobotAction,
    RobotObservation,
    RobotProcessorPipeline,
    make_default_processors,
)
from lerobot.robots import Robot, RobotConfig, make_robot_from_config
from lerobot.teleoperators import Teleoperator, TeleoperatorConfig, make_teleoperator_from_config
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import init_logging, move_cursor_up
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data


@dataclass
class Pico4TeleoperateConfig:
    teleop: TeleoperatorConfig
    robot: RobotConfig
    fps: int = 60
    teleop_time_s: float | None = None
    display_data: bool = False
    display_ip: str | None = None
    display_port: int | None = None
    display_compressed_images: bool = False


def sync_teleop_tcp_pose(teleop: Teleoperator, robot: Robot) -> None:
    if not hasattr(teleop, "set_current_tcp_pose"):
        raise ValueError(f"{teleop} does not support current TCP pose synchronization.")
    if not hasattr(robot, "get_current_tcp_pose_quat"):
        raise ValueError(f"{robot} does not provide get_current_tcp_pose_quat().")
    teleop.set_current_tcp_pose(robot.get_current_tcp_pose_quat())


def reset_to_initial_position(robot: Robot, teleop: Teleoperator) -> None:
    if hasattr(robot, "reset_to_initial_position"):
        robot.reset_to_initial_position()
    elif hasattr(robot, "_return_to_initial_position"):
        robot._return_to_initial_position()
    else:
        raise ValueError(f"{robot} does not provide reset_to_initial_position().")

    sync_teleop_tcp_pose(teleop, robot)
    if hasattr(teleop, "reset_to_current_tcp_pose"):
        teleop.reset_to_current_tcp_pose()


def teleop_loop(
    teleop: Teleoperator,
    robot: Robot,
    fps: int,
    teleop_action_processor: RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction],
    robot_action_processor: RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction],
    robot_observation_processor: RobotProcessorPipeline[RobotObservation, RobotObservation],
    display_data: bool = False,
    duration: float | None = None,
    display_compressed_images: bool = False,
) -> None:
    display_len = max(len(key) for key in robot.action_features)
    start = time.perf_counter()

    while True:
        loop_start = time.perf_counter()

        obs = robot.get_observation()
        sync_teleop_tcp_pose(teleop, robot)

        raw_action = teleop.get_action()
        if hasattr(teleop, "get_reset_button") and teleop.get_reset_button():
            try:
                logging.info("Reset to initial position (A button pressed).")
                reset_to_initial_position(robot, teleop)
            except Exception as e:
                logging.error("Failed to reset robot position: %s\n%s", e, traceback.format_exc())

            if display_data:
                obs_transition = robot_observation_processor(obs)
                log_rerun_data(
                    observation=obs_transition,
                    action={},
                    compress_images=display_compressed_images,
                )
            continue

        teleop_action = teleop_action_processor((raw_action, obs))
        robot_action_to_send = robot_action_processor((teleop_action, obs))
        sent_action = robot.send_action(robot_action_to_send)

        if display_data:
            obs_transition = robot_observation_processor(obs)

            log_rerun_data(
                observation=obs_transition,
                action=sent_action,
                compress_images=display_compressed_images,
            )

            print("\n" + "-" * (display_len + 10))
            print(f"{'NAME':<{display_len}} | {'NORM':>7}")
            for motor, value in sent_action.items():
                print(f"{motor:<{display_len}} | {value:>7.2f}")
            move_cursor_up(len(sent_action) + 3)

        dt_s = time.perf_counter() - loop_start
        precise_sleep(max(1 / fps - dt_s, 0.0))
        loop_s = time.perf_counter() - loop_start
        print(f"Teleop loop time: {loop_s * 1e3:.2f}ms ({1 / loop_s:.0f} Hz)")
        move_cursor_up(1)

        if duration is not None and time.perf_counter() - start >= duration:
            return


@parser.wrap()
def teleoperate_pico4(cfg: Pico4TeleoperateConfig) -> None:
    init_logging()
    logging.info(pformat(asdict(cfg)))

    if cfg.teleop.type != "pico4":
        raise ValueError("lerobot-teleoperate-pico4 requires --teleop.type=pico4.")
    if getattr(cfg.robot, "action_mode", None) != "cartesian":
        raise ValueError(
            "Pico4 teleoperation requires --robot.action_mode=cartesian so tcp.* actions are accepted."
        )

    if cfg.display_data:
        init_rerun(session_name="pico4_teleoperation", ip=cfg.display_ip, port=cfg.display_port)
    display_compressed_images = (
        True
        if (cfg.display_data and cfg.display_ip is not None and cfg.display_port is not None)
        else cfg.display_compressed_images
    )

    teleop = make_teleoperator_from_config(cfg.teleop)
    robot = make_robot_from_config(cfg.robot)
    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

    try:
        robot.connect()
        sync_teleop_tcp_pose(teleop, robot)
        logging.info("Start TCP pose (quat): %s", robot.get_current_tcp_pose_quat())

        teleop.connect()

        teleop_loop(
            teleop=teleop,
            robot=robot,
            fps=cfg.fps,
            display_data=cfg.display_data,
            duration=cfg.teleop_time_s,
            teleop_action_processor=teleop_action_processor,
            robot_action_processor=robot_action_processor,
            robot_observation_processor=robot_observation_processor,
            display_compressed_images=display_compressed_images,
        )
    except KeyboardInterrupt:
        pass
    finally:
        if cfg.display_data:
            rr.rerun_shutdown()
        try:
            if teleop.is_connected:
                teleop.disconnect()
        finally:
            if robot.is_connected:
                robot.disconnect()


def main() -> None:
    register_third_party_plugins()
    teleoperate_pico4()
