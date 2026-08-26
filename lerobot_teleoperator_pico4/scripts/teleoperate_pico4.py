import logging
import time
import traceback
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict, dataclass
from pprint import pformat

import rerun as rr

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

# Built-in robots (unlike entry-point plugins) are only registered with draccus when
# their config module is imported. Import TRON2 so ``--robot.type=tron2`` is a valid
# choice; guard it so the plugin still works against a lerobot without TRON2.
try:
    from lerobot.robots import tron2 as _tron2  # noqa: F401
except ImportError:
    pass
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import init_logging, move_cursor_up
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data

from ..action_compatibility import check_teleop_robot_action_compatibility


PICO4_TELEOP_TYPES = {"pico4", "bi_pico4", "pico4head", "bi_pico4_head"}


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


def connect_robot_and_preinitialize_teleop(teleop: Teleoperator, robot: Robot) -> None:
    """Overlap robot connection with optional Pico SDK pre-initialization."""
    pre_init = getattr(teleop, "pre_init", None)
    if not callable(pre_init):
        robot.connect()
        return

    try:
        with ThreadPoolExecutor(max_workers=2) as executor:
            robot_future = executor.submit(robot.connect)
            teleop_future = executor.submit(pre_init)
            teleop_future.result()
            robot_future.result()
    except Exception:
        xrt = getattr(teleop, "_xrt", None)
        if xrt is not None and not teleop.is_connected:
            try:
                xrt.close()
            finally:
                teleop._xrt = None
        raise


def sync_teleop_tcp_pose(teleop: Teleoperator, robot: Robot) -> None:
    if not getattr(teleop, "requires_current_tcp_pose", False):
        return

    if not hasattr(robot, "get_current_tcp_pose_quat"):
        raise ValueError(f"{robot} does not provide get_current_tcp_pose_quat().")

    current_pose = robot.get_current_tcp_pose_quat()
    if teleop.name == "bi_pico4_head":
        teleop.set_current_tcp_pose(current_pose)
    elif hasattr(teleop, "set_current_tcp_poses"):
        left_pose, right_pose = current_pose
        teleop.set_current_tcp_poses(left_pose, right_pose)
    elif hasattr(teleop, "set_current_tcp_pose"):
        teleop.set_current_tcp_pose(current_pose)
    else:
        raise ValueError(f"{teleop} does not support current TCP pose synchronization.")


def connect_teleop_with_robot_pose(teleop: Teleoperator, robot: Robot) -> None:
    if not getattr(teleop, "requires_current_tcp_pose", False):
        teleop.connect()
        return

    current_pose = robot.get_current_tcp_pose_quat()
    if teleop.name == "bi_pico4_head":
        left_pose, right_pose, head_pose = current_pose
        logging.info("Start left TCP pose (quat): %s", left_pose)
        logging.info("Start right TCP pose (quat): %s", right_pose)
        logging.info("Start head TCP pose (quat): %s", head_pose)
        teleop.connect(
            left_tcp_pose_quat=left_pose,
            right_tcp_pose_quat=right_pose,
            head_tcp_pose_quat=head_pose,
        )
    elif hasattr(teleop, "set_current_tcp_poses"):
        left_pose, right_pose = current_pose
        logging.info("Start left TCP pose (quat): %s", left_pose)
        logging.info("Start right TCP pose (quat): %s", right_pose)
        teleop.connect(left_tcp_pose_quat=left_pose, right_tcp_pose_quat=right_pose)
    else:
        if hasattr(teleop, "set_current_tcp_pose"):
            teleop.set_current_tcp_pose(current_pose)
        logging.info("Start TCP pose (quat): %s", current_pose)
        teleop.connect()


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

    if cfg.teleop.type not in PICO4_TELEOP_TYPES:
        raise ValueError(
            "lerobot-teleoperate-pico4 requires "
            "--teleop.type=pico4, bi_pico4, pico4head, or bi_pico4_head."
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
    check_teleop_robot_action_compatibility(teleop, robot)
    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

    try:
        connect_robot_and_preinitialize_teleop(teleop, robot)
        connect_teleop_with_robot_pose(teleop, robot)

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
