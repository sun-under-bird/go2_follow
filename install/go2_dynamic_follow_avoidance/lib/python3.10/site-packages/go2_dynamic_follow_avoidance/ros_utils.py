import importlib
import math
from typing import Any, Iterable, Optional

from rclpy.node import Node


def import_message_type(type_name: str):
    """Import a ROS message type from strings like package/msg/Type."""
    parts = type_name.split("/")
    if len(parts) != 3 or parts[1] != "msg":
        raise ValueError(f"message type must look like package/msg/Type, got {type_name!r}")
    module = importlib.import_module(f"{parts[0]}.msg")
    return getattr(module, parts[2])


def get_nested_attr(obj: Any, dotted_name: str, default: Any = None) -> Any:
    current = obj
    for part in dotted_name.split("."):
        if not hasattr(current, part):
            return default
        current = getattr(current, part)
    return current


def first_existing_attr(obj: Any, names: Iterable[str], default: Any = None) -> Any:
    for name in names:
        value = get_nested_attr(obj, name, None)
        if value is not None:
            return value
    return default


def to_float(value: Any, default: Optional[float] = None) -> Optional[float]:
    try:
        if value is None:
            return default
        result = float(value)
        if math.isfinite(result):
            return result
    except (TypeError, ValueError):
        pass
    return default


def lookup_transform_or_none(node: Node, tf_buffer, target_frame: str, source_frame: str):
    if target_frame == source_frame:
        return None
    try:
        return tf_buffer.lookup_transform(target_frame, source_frame, rclpy_time_zero())
    except Exception as exc:  # noqa: BLE001 - TF failures are expected during startup.
        node.get_logger().debug(
            f"TF lookup failed from {source_frame} to {target_frame}: {exc}"
        )
        return None


def rclpy_time_zero():
    from rclpy.time import Time

    return Time()
