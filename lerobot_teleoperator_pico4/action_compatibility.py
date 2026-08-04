from typing import Any


def _teleop_action_feature_names(teleop: Any) -> set[str]:
    features = teleop.action_features
    if isinstance(features, dict) and isinstance(features.get("names"), dict):
        return set(features["names"])
    return set(features)


def check_teleop_robot_action_compatibility(teleop: Any, robot: Any) -> None:
    """Require exact action-key compatibility without binding to robot type names."""
    teleop_features = _teleop_action_feature_names(teleop)
    robot_features = set(robot.action_features)
    missing_in_robot = sorted(teleop_features - robot_features)
    extra_in_robot = sorted(robot_features - teleop_features)
    if missing_in_robot or extra_in_robot:
        details: list[str] = []
        if missing_in_robot:
            details.append(f"robot is missing {missing_in_robot}")
        if extra_in_robot:
            details.append(f"robot has extra fields {extra_in_robot}")
        raise ValueError(
            f"Action features for teleop {teleop.name!r} and robot {robot.name!r} "
            f"are incompatible: {'; '.join(details)}."
        )
