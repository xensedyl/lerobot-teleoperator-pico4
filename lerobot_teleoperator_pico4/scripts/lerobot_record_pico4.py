import logging
import threading
import time
import traceback
from dataclasses import asdict, dataclass, field
from pathlib import Path
from pprint import pformat

import rerun as rr

from lerobot.cameras import CameraConfig  # noqa: F401
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig  # noqa: F401
from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig  # noqa: F401
from lerobot.configs import parser
from lerobot.datasets.image_writer import safe_stop_image_writer
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.pipeline_features import aggregate_pipeline_dataset_features, create_initial_features
from lerobot.datasets.utils import build_dataset_frame, combine_feature_dicts
from lerobot.datasets.video_utils import VideoEncodingManager
from lerobot.processor import (
    RobotAction,
    RobotObservation,
    RobotProcessorPipeline,
    make_default_processors,
)
from lerobot.robots import Robot, RobotConfig, make_robot_from_config
from lerobot.teleoperators import Teleoperator, TeleoperatorConfig, make_teleoperator_from_config
from lerobot.utils.constants import ACTION, OBS_STR
from lerobot.utils.control_utils import (
    init_keyboard_listener,
    is_headless,
    sanity_check_dataset_name,
    sanity_check_dataset_robot_compatibility,
)
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import init_logging, log_say
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data

from .teleoperate_pico4 import connect_teleop_with_robot_pose, reset_to_initial_position, sync_teleop_tcp_pose


@dataclass
class Pico4DatasetRecordConfig:
    repo_id: str
    single_task: str
    root: str | Path | None = None
    fps: int = 30
    episode_time_s: int | float = 60
    reset_time_s: int | float = 60
    num_episodes: int = 50
    video: bool = True
    push_to_hub: bool = True
    private: bool = False
    tags: list[str] | None = None
    num_image_writer_processes: int = 0
    num_image_writer_threads_per_camera: int = 4
    video_encoding_batch_size: int = 1
    vcodec: str = "libsvtav1"
    rename_map: dict[str, str] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.single_task is None:
            raise ValueError("You need to provide --dataset.single_task.")


@dataclass
class Pico4RecordConfig:
    robot: RobotConfig
    teleop: TeleoperatorConfig
    dataset: Pico4DatasetRecordConfig
    display_data: bool = False
    display_ip: str | None = None
    display_port: int | None = None
    display_compressed_images: bool = False
    play_sounds: bool = True
    resume: bool = False


def _gripper_observation_to_action(robot: Robot, obs: RobotObservation, key: str) -> float:
    if key in obs:
        return max(0.0, min(1.0, float(obs[key])))

    current_pose = robot.get_current_tcp_pose_quat()
    if key == "gripper.pos":
        return float(current_pose[7])
    if key == "left_gripper.pos":
        return float(current_pose[0][7])
    if key == "right_gripper.pos":
        return float(current_pose[1][7])
    raise KeyError(key)


def _observation_as_action(robot: Robot, obs: RobotObservation) -> RobotAction:
    action: RobotAction = {}
    for key in robot.action_features:
        if key.endswith("gripper.pos"):
            action[key] = _gripper_observation_to_action(robot, obs, key)
        elif key in obs:
            action[key] = obs[key]

    missing = set(robot.action_features) - set(action)
    if missing:
        raise ValueError(f"Cannot build reset action from observation; missing keys: {sorted(missing)}")
    return action


def _start_reset_in_background(
    robot: Robot,
    teleop: Teleoperator,
    reset_done: threading.Event,
) -> threading.Thread:
    def _run_reset() -> None:
        try:
            logging.info("Reset to initial position (A button pressed).")
            reset_to_initial_position(robot, teleop)
        except Exception as e:
            logging.error("Failed to reset robot position: %s\n%s", e, traceback.format_exc())
        finally:
            reset_done.set()

    reset_done.clear()
    thread = threading.Thread(target=_run_reset, daemon=True)
    thread.start()
    return thread


@safe_stop_image_writer
def record_loop(
    robot: Robot,
    events: dict,
    fps: int,
    teleop_action_processor: RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction],
    robot_action_processor: RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction],
    robot_observation_processor: RobotProcessorPipeline[RobotObservation, RobotObservation],
    dataset: LeRobotDataset | None,
    teleop: Teleoperator,
    control_time_s: int | float,
    single_task: str,
    display_data: bool = False,
    display_compressed_images: bool = False,
) -> None:
    if dataset is not None and dataset.fps != fps:
        raise ValueError(f"The dataset fps should be equal to requested fps ({dataset.fps} != {fps}).")

    timestamp = 0.0
    start_episode_t = time.perf_counter()
    reset_done = threading.Event()
    reset_done.set()
    reset_thread: threading.Thread | None = None
    prev_observation_frame = None
    try:
        while timestamp < control_time_s:
            start_loop_t = time.perf_counter()
            reset_triggered = False

            if events["exit_early"]:
                events["exit_early"] = False
                break

            resetting = not reset_done.is_set()
            obs = robot.get_observation()
            sync_teleop_tcp_pose(teleop, robot)

            obs_processed = robot_observation_processor(obs)
            observation_frame = None
            if dataset is not None:
                observation_frame = build_dataset_frame(dataset.features, obs_processed, prefix=OBS_STR)

            if resetting:
                action_values = _observation_as_action(robot, obs)
            else:
                raw_action = teleop.get_action()
                if hasattr(teleop, "get_reset_button") and teleop.get_reset_button():
                    reset_thread = _start_reset_in_background(robot, teleop, reset_done)
                    reset_triggered = True
                    action_values = _observation_as_action(robot, obs)
                else:
                    action_values = teleop_action_processor((raw_action, obs))
                    robot_action_to_send = robot_action_processor((action_values, obs))
                    action_values = robot.send_action(robot_action_to_send)

            if dataset is not None:
                action_frame = build_dataset_frame(dataset.features, action_values, prefix=ACTION)
                if (resetting or reset_triggered) and prev_observation_frame is not None:
                    # Match BiFlexivRT reset recording: the reset runs in the robot
                    # thread, so action[t] is the current robot state reached from
                    # obs[t-1], not a Pico4 command sent by Python.
                    dataset.add_frame({**prev_observation_frame, **action_frame, "task": single_task})
                elif not (resetting or reset_triggered):
                    dataset.add_frame({**observation_frame, **action_frame, "task": single_task})

                prev_observation_frame = observation_frame

            if display_data:
                log_rerun_data(
                    observation=obs_processed,
                    action=action_values,
                    compress_images=display_compressed_images,
                )

            dt_s = time.perf_counter() - start_loop_t
            precise_sleep(max(1 / fps - dt_s, 0.0))
            timestamp = time.perf_counter() - start_episode_t
    finally:
        if reset_thread is not None and reset_thread.is_alive():
            reset_thread.join()


@parser.wrap()
def record_pico4(cfg: Pico4RecordConfig) -> LeRobotDataset:
    init_logging()
    logging.info(pformat(asdict(cfg)))

    if cfg.teleop.type not in {"pico4", "bi_pico4"}:
        raise ValueError("lerobot-record-pico4 requires --teleop.type=pico4 or bi_pico4.")
    if getattr(cfg.robot, "action_mode", None) != "cartesian":
        raise ValueError("Pico4 recording requires --robot.action_mode=cartesian.")
    if cfg.teleop.type == "bi_pico4" and cfg.robot.type != "bi_seeed_b601_rt_follower":
        raise ValueError("--teleop.type=bi_pico4 requires --robot.type=bi_seeed_b601_rt_follower.")
    if cfg.teleop.type == "pico4" and cfg.robot.type == "bi_seeed_b601_rt_follower":
        raise ValueError("--robot.type=bi_seeed_b601_rt_follower requires --teleop.type=bi_pico4.")

    if cfg.display_data:
        init_rerun(session_name="pico4_recording", ip=cfg.display_ip, port=cfg.display_port)
    display_compressed_images = (
        True
        if (cfg.display_data and cfg.display_ip is not None and cfg.display_port is not None)
        else cfg.display_compressed_images
    )

    robot = make_robot_from_config(cfg.robot)
    teleop = make_teleoperator_from_config(cfg.teleop)
    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

    dataset_features = combine_feature_dicts(
        aggregate_pipeline_dataset_features(
            pipeline=teleop_action_processor,
            initial_features=create_initial_features(action=robot.action_features),
            use_videos=cfg.dataset.video,
        ),
        aggregate_pipeline_dataset_features(
            pipeline=robot_observation_processor,
            initial_features=create_initial_features(observation=robot.observation_features),
            use_videos=cfg.dataset.video,
        ),
    )

    dataset = None
    listener = None
    try:
        if cfg.resume:
            dataset = LeRobotDataset(
                cfg.dataset.repo_id,
                root=cfg.dataset.root,
                batch_encoding_size=cfg.dataset.video_encoding_batch_size,
                vcodec=cfg.dataset.vcodec,
            )
            if hasattr(robot, "cameras") and len(robot.cameras) > 0:
                dataset.start_image_writer(
                    num_processes=cfg.dataset.num_image_writer_processes,
                    num_threads=cfg.dataset.num_image_writer_threads_per_camera * len(robot.cameras),
                )
            sanity_check_dataset_robot_compatibility(
                dataset, robot, cfg.dataset.fps, dataset_features
            )
        else:
            sanity_check_dataset_name(cfg.dataset.repo_id, None)
            dataset = LeRobotDataset.create(
                cfg.dataset.repo_id,
                cfg.dataset.fps,
                root=cfg.dataset.root,
                robot_type=robot.name,
                features=dataset_features,
                use_videos=cfg.dataset.video,
                image_writer_processes=cfg.dataset.num_image_writer_processes,
                image_writer_threads=cfg.dataset.num_image_writer_threads_per_camera * len(robot.cameras),
                batch_encoding_size=cfg.dataset.video_encoding_batch_size,
                vcodec=cfg.dataset.vcodec,
            )

        robot.connect()
        connect_teleop_with_robot_pose(teleop, robot)

        listener, events = init_keyboard_listener()

        with VideoEncodingManager(dataset):
            recorded_episodes = 0
            while recorded_episodes < cfg.dataset.num_episodes and not events["stop_recording"]:
                log_say(f"Recording episode {dataset.num_episodes}", cfg.play_sounds)
                record_loop(
                    robot=robot,
                    events=events,
                    fps=cfg.dataset.fps,
                    teleop_action_processor=teleop_action_processor,
                    robot_action_processor=robot_action_processor,
                    robot_observation_processor=robot_observation_processor,
                    teleop=teleop,
                    dataset=dataset,
                    control_time_s=cfg.dataset.episode_time_s,
                    single_task=cfg.dataset.single_task,
                    display_data=cfg.display_data,
                    display_compressed_images=display_compressed_images,
                )

                if not events["stop_recording"] and (
                    (recorded_episodes < cfg.dataset.num_episodes - 1) or events["rerecord_episode"]
                ):
                    log_say("Reset the environment", cfg.play_sounds)
                    record_loop(
                        robot=robot,
                        events=events,
                        fps=cfg.dataset.fps,
                        teleop_action_processor=teleop_action_processor,
                        robot_action_processor=robot_action_processor,
                        robot_observation_processor=robot_observation_processor,
                        teleop=teleop,
                        dataset=None,
                        control_time_s=cfg.dataset.reset_time_s,
                        single_task=cfg.dataset.single_task,
                        display_data=cfg.display_data,
                        display_compressed_images=display_compressed_images,
                    )

                if events["rerecord_episode"]:
                    log_say("Re-record episode", cfg.play_sounds)
                    events["rerecord_episode"] = False
                    events["exit_early"] = False
                    dataset.clear_episode_buffer()
                    continue

                dataset.save_episode()
                recorded_episodes += 1
    finally:
        log_say("Stop recording", cfg.play_sounds, blocking=True)
        if dataset:
            dataset.finalize()
        if robot.is_connected:
            robot.disconnect()
        if teleop and teleop.is_connected:
            teleop.disconnect()
        if not is_headless() and listener:
            listener.stop()
        if cfg.display_data:
            rr.rerun_shutdown()
        if dataset and cfg.dataset.push_to_hub:
            dataset.push_to_hub(tags=cfg.dataset.tags, private=cfg.dataset.private)
        log_say("Exiting", cfg.play_sounds)

    return dataset


def main() -> None:
    register_third_party_plugins()
    record_pico4()
