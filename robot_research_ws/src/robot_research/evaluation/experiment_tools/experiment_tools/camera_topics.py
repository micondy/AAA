import re


VALID_NAME = re.compile(r"^[A-Za-z0-9_]+$")
VALID_STAGES = ("basic", "imu", "aligned", "full")


def normalize_name(value: str, label: str) -> str:
    value = value.strip().strip("/")
    if not value or not VALID_NAME.fullmatch(value):
        raise ValueError(f"{label} must contain only letters, digits, and underscores")
    return value


def normalize_namespace(value: str) -> str:
    value = value.strip().strip("/")
    parts = value.split("/") if value else []
    if not parts or any(not VALID_NAME.fullmatch(part) for part in parts):
        raise ValueError(
            "camera_namespace must contain slash-separated letters, digits, and underscores"
        )
    return "/".join(parts)


def camera_prefix(camera_namespace: str, camera_name: str) -> str:
    namespace = normalize_namespace(camera_namespace)
    name = normalize_name(camera_name, "camera_name")
    return f"/{namespace}/{name}"


def camera_topics(camera_namespace: str, camera_name: str, stage: str) -> list[str]:
    stage = stage.strip().lower()
    if stage not in VALID_STAGES:
        raise ValueError(f"stage must be one of: {', '.join(VALID_STAGES)}")

    prefix = camera_prefix(camera_namespace, camera_name)
    depth_prefix = (
        f"{prefix}/aligned_depth_to_color"
        if stage in ("aligned", "full")
        else f"{prefix}/depth"
    )
    topics = [
        f"{prefix}/color/image_raw",
        f"{prefix}/color/camera_info",
        f"{depth_prefix}/image_raw" if stage in ("aligned", "full")
        else f"{depth_prefix}/image_rect_raw",
        f"{depth_prefix}/camera_info",
    ]
    if stage in ("imu", "aligned", "full"):
        topics.extend([
            f"{prefix}/imu",
            f"{prefix}/gyro/sample",
            f"{prefix}/accel/sample",
        ])
    topics.extend(["/diagnostics", "/tf", "/tf_static"])
    return topics


def depth_topic(camera_namespace: str, camera_name: str, stage: str) -> str:
    topics = camera_topics(camera_namespace, camera_name, stage)
    return topics[2]
