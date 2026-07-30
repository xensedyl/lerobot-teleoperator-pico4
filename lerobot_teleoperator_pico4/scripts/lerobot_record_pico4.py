import contextlib
import logging
import threading
import time
import traceback
from dataclasses import asdict, dataclass, field
from fractions import Fraction
from pathlib import Path
from pprint import pformat
from tempfile import TemporaryDirectory

import rerun as rr

from lerobot.cameras import CameraConfig  # noqa: F401
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig  # noqa: F401
from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig  # noqa: F401
from lerobot.configs import parser
from lerobot.configs.video import RGBEncoderConfig
from lerobot.datasets.image_writer import safe_stop_image_writer
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.pipeline_features import aggregate_pipeline_dataset_features, create_initial_features
from lerobot.datasets.video_utils import VideoEncodingManager
from lerobot.processor import (
    RobotAction,
    RobotObservation,
    RobotProcessorPipeline,
    make_default_processors,
)
from lerobot.robots import Robot, RobotConfig, make_robot_from_config
from lerobot.utils.feature_utils import build_dataset_frame, combine_feature_dicts

# Built-in robots are only registered with draccus when their config module is
# imported. Import TRON2 so ``--robot.type=tron2`` is a valid choice; guard it so
# the plugin still works against a lerobot without TRON2.
try:
    from lerobot.robots import tron2 as _tron2  # noqa: F401
except ImportError:
    pass
from lerobot.common.control_utils import (
    sanity_check_dataset_name,
    sanity_check_dataset_robot_compatibility,
)
from lerobot.teleoperators import Teleoperator, TeleoperatorConfig, make_teleoperator_from_config
from lerobot.utils.constants import ACTION, HF_LEROBOT_HOME, OBS_STR
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.keyboard_input import init_keyboard_listener, is_headless
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import init_logging, log_say
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data

from .teleoperate_pico4 import (
    BIMANUAL_ROBOTS,
    connect_teleop_with_robot_pose,
    reset_to_initial_position,
    sync_teleop_tcp_pose,
)


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
    streaming_encoding: bool = True
    encoder_queue_maxsize: int = 30
    encoder_threads: int | None = None
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
    camera_stabilization_time_s: float = 2.0


def _probe_rgb_encoder(
    encoder: RGBEncoderConfig,
    fps: int,
    encoder_threads: int | None,
) -> None:
    """Open the codec and encode one frame so hardware-only failures happen before recording."""
    import av
    import numpy as np

    with TemporaryDirectory() as tmp_dir:
        container = av.open(str(Path(tmp_dir) / "encoder_probe.mp4"), "w")
        try:
            stream = container.add_stream(
                encoder.vcodec,
                fps,
                options=encoder.get_codec_options(encoder_threads, as_strings=True),
            )
            stream.pix_fmt = encoder.pix_fmt
            stream.width = 64
            stream.height = 64
            stream.time_base = Fraction(1, fps)

            frame = av.VideoFrame.from_ndarray(np.zeros((64, 64, 3), dtype=np.uint8), format="rgb24")
            frame.pts = 0
            frame.time_base = Fraction(1, fps)
            for packet in stream.encode(frame):
                container.mux(packet)
            for packet in stream.encode():
                container.mux(packet)
        finally:
            with contextlib.suppress(Exception):
                container.close()


def _make_rgb_encoder(
    requested_vcodec: str,
    fps: int,
    encoder_threads: int | None,
) -> RGBEncoderConfig:
    encoder = RGBEncoderConfig(vcodec=requested_vcodec)
    try:
        _probe_rgb_encoder(encoder, fps, encoder_threads)
        return encoder
    except Exception as error:
        if requested_vcodec != "auto":
            raise RuntimeError(
                f"Video encoder {encoder.vcodec!r} could not be opened. "
                "Try --dataset.vcodec=h264 or --dataset.vcodec=libsvtav1."
            ) from error

        logging.warning(
            "Auto-selected video encoder %s could not be opened (%s). Trying software fallback.",
            encoder.vcodec,
            error,
        )

    fallback_errors: list[str] = []
    for fallback_vcodec in ("h264", "libsvtav1"):
        if fallback_vcodec == encoder.vcodec:
            continue
        try:
            fallback = RGBEncoderConfig(vcodec=fallback_vcodec)
            _probe_rgb_encoder(fallback, fps, encoder_threads)
            logging.warning("Falling back to video encoder %s.", fallback.vcodec)
            return fallback
        except Exception as error:
            fallback_errors.append(f"{fallback_vcodec}: {error}")

    raise RuntimeError("No usable RGB video encoder found: " + "; ".join(fallback_errors))


def _wait_for_stable_camera_frames(
    robot: Robot,
    fps: int,
    stabilization_time_s: float,
) -> None:
    """Require continuous fresh frames from every camera before recording."""
    cameras = getattr(robot, "cameras", {})
    if not cameras or stabilization_time_s <= 0:
        return

    camera_configs = getattr(getattr(robot, "config", None), "cameras", {})
    required_cycles = max(1, round(fps * stabilization_time_s))
    stabilization_deadline = time.monotonic() + max(10.0, stabilization_time_s * 3)
    stable_cycles = 0
    logging.info(
        "Waiting for %d camera stream(s) to remain stable for %.1f seconds before recording.",
        len(cameras),
        stabilization_time_s,
    )

    while stable_cycles < required_cycles:
        try:
            for name, camera in cameras.items():
                async_read = getattr(camera, "async_read", None)
                if not callable(async_read):
                    continue
                camera_config = camera_configs.get(name) if isinstance(camera_configs, dict) else None
                timeout_ms = max(int(getattr(camera_config, "frame_timeout_ms", 1000)), 1000)
                async_read(timeout_ms=timeout_ms)
            stable_cycles += 1
        except TimeoutError as error:
            stable_cycles = 0
            if time.monotonic() >= stabilization_deadline:
                raise TimeoutError(
                    f"Camera streams did not remain stable for {stabilization_time_s:.1f} seconds "
                    "before the warmup deadline."
                ) from error
            logging.warning("Camera %r warmup was interrupted; restarting the stability window.", name)

    # Validate the same integrated observation path used by record_loop without
    # adding the warmup sample to the dataset.
    robot.get_observation()
    logging.info("Camera streams are stable; starting episode recording.")


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


def _disconnect_recording_devices(robot: Robot, teleop: Teleoperator | None) -> None:
    if robot.is_connected:
        robot.disconnect()
    if teleop is not None and teleop.is_connected:
        teleop.disconnect()


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
    # TRON2 is inherently Cartesian (tcp.* actions); other robots must opt in via action_mode.
    if cfg.robot.type != "tron2" and getattr(cfg.robot, "action_mode", None) != "cartesian":
        raise ValueError("Pico4 recording requires --robot.action_mode=cartesian.")
    if cfg.teleop.type == "bi_pico4" and cfg.robot.type not in BIMANUAL_ROBOTS:
        raise ValueError(
            "--teleop.type=bi_pico4 requires a bimanual robot "
            "(--robot.type=bi_seeed_b601_rt_follower or tron2)."
        )
    if cfg.teleop.type == "pico4" and cfg.robot.type in BIMANUAL_ROBOTS:
        raise ValueError(f"--robot.type={cfg.robot.type} requires --teleop.type=bi_pico4.")

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
    rgb_encoder = (
        _make_rgb_encoder(cfg.dataset.vcodec, cfg.dataset.fps, cfg.dataset.encoder_threads)
        if cfg.dataset.video
        else None
    )

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
            resume_root = (
                Path(cfg.dataset.root)
                if cfg.dataset.root is not None
                else HF_LEROBOT_HOME / cfg.dataset.repo_id
            )
            num_cameras = len(robot.cameras) if hasattr(robot, "cameras") else 0
            dataset = LeRobotDataset.resume(
                cfg.dataset.repo_id,
                root=resume_root,
                batch_encoding_size=cfg.dataset.video_encoding_batch_size,
                rgb_encoder=rgb_encoder,
                streaming_encoding=cfg.dataset.streaming_encoding,
                encoder_queue_maxsize=cfg.dataset.encoder_queue_maxsize,
                encoder_threads=cfg.dataset.encoder_threads,
                image_writer_processes=cfg.dataset.num_image_writer_processes if num_cameras > 0 else 0,
                image_writer_threads=(
                    cfg.dataset.num_image_writer_threads_per_camera * num_cameras if num_cameras > 0 else 0
                ),
            )
            sanity_check_dataset_robot_compatibility(dataset, robot, cfg.dataset.fps, dataset_features)
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
                rgb_encoder=rgb_encoder,
                streaming_encoding=cfg.dataset.streaming_encoding,
                encoder_queue_maxsize=cfg.dataset.encoder_queue_maxsize,
                encoder_threads=cfg.dataset.encoder_threads,
            )

        robot.connect()
        connect_teleop_with_robot_pose(teleop, robot)

        listener, events = init_keyboard_listener()

        if not cfg.dataset.streaming_encoding:
            logging.info(
                "Streaming encoding is disabled. Enable it with "
                "--dataset.streaming_encoding=true to encode camera frames while recording."
            )

        with VideoEncodingManager(dataset):
            recorded_episodes = 0
            while recorded_episodes < cfg.dataset.num_episodes and not events["stop_recording"]:
                # Some LeRobot versions can initialize codec contexts before the
                # first frame. Other versions start them from the first add_frame().
                prepare_episode_recording = getattr(dataset, "prepare_episode_recording", None)
                if callable(prepare_episode_recording):
                    prepare_episode_recording()

                _wait_for_stable_camera_frames(
                    robot,
                    fps=cfg.dataset.fps,
                    stabilization_time_s=cfg.camera_stabilization_time_s,
                )
                if events["stop_recording"]:
                    break

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

                if recorded_episodes >= cfg.dataset.num_episodes - 1 or events["stop_recording"]:
                    _disconnect_recording_devices(robot, teleop)

                dataset.save_episode()
                recorded_episodes += 1
    finally:
        log_say("Stop recording", cfg.play_sounds, blocking=True)
        _disconnect_recording_devices(robot, teleop)
        if dataset:
            dataset.finalize()
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
