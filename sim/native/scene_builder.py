"""Native scene builder for the Nero arm body package.

Resolves the robot model entry and runtime asset list from the body
package's `assets/robots/<id>/{robot.json,index.json}` and hands the
Native runtime an absolute path to load. The Web loader copies these
same assets into MuJoCo MEMFS; for the Native runtime the assets are
already on the local filesystem, so the builder validates them in place
rather than copying to a temp dir.

The robot registry (`assets/robots/index.json`) and the per-robot
`robot.json` remain the single authority for model entry, asset list and
sensor configuration, per the onboarding guide.
"""

from __future__ import annotations

import json
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parent.parent.parent
ROBOTS_DIR = PACKAGE_ROOT / "assets" / "robots"
SCENES_DIR = PACKAGE_ROOT / "assets" / "scenes"


class SceneBuildError(RuntimeError):
    """Raised when a robot's assets are missing or malformed."""


def _load_json(path: Path) -> dict:
    try:
        with path.open("r", encoding="utf-8") as fh:
            return json.load(fh)
    except FileNotFoundError as exc:
        raise SceneBuildError(f"missing file: {path}") from exc
    except json.JSONDecodeError as exc:
        raise SceneBuildError(f"invalid JSON in {path}: {exc}") from exc


def _validate_relative(entry: str) -> None:
    """Onboarding rule: index entries are robot-dir-relative, no absolute
    paths, no `..` traversal."""
    p = Path(entry)
    if p.is_absolute() or ".." in p.parts:
        raise SceneBuildError(
            f"index entry {entry!r} must be a relative path with no '..'"
        )


def default_robot_id() -> str:
    registry = _load_json(ROBOTS_DIR / "index.json")
    robot_id = registry.get("defaultRobot")
    if not robot_id:
        raise SceneBuildError("assets/robots/index.json missing 'defaultRobot'")
    return robot_id


def default_scene_id() -> str | None:
    """The scene registry's default scene id, or None if no scenes are shipped."""
    index = SCENES_DIR / "index.json"
    if not index.is_file():
        return None
    return _load_json(index).get("defaultScene")


def _resolve_scene_path(scene_id: str) -> str:
    """Resolve a scene id to its XML path (``assets/scenes/<id>.xml``)."""
    if not scene_id or "/" in scene_id or "\\" in scene_id or ".." in scene_id:
        raise SceneBuildError(f"invalid scene id {scene_id!r}")
    path = SCENES_DIR / f"{scene_id}.xml"
    if not path.is_file():
        raise SceneBuildError(f"scene model missing on disk: {path}")
    return str(path)


def build_scene(robot_id: str | None = None, scene_id: str | None = None) -> dict:
    """Return a dict describing the resolved scene:
        {"robot_id", "scene_id", "model_path", "assets": [...], "controlled_actuators": [...]}

    ``model_path`` is the XML the runtime loads: the scene XML when a scene is
    resolved (the default), otherwise the bare arm wrapper. ``scene_id`` is None
    when no scene was selected.
    """
    robot_id = robot_id or default_robot_id()
    robot_dir = ROBOTS_DIR / robot_id
    if not robot_dir.is_dir():
        raise SceneBuildError(f"no robot directory for id {robot_id!r} at {robot_dir}")

    robot_json = _load_json(robot_dir / "robot.json")
    index_path = robot_dir / robot_json.get("files", "index.json")
    assets = _load_json(index_path)
    if not isinstance(assets, list):
        raise SceneBuildError(f"{index_path} must contain a JSON array of asset paths")

    for entry in assets:
        _validate_relative(entry)
        if not (robot_dir / entry).is_file():
            raise SceneBuildError(f"asset missing on disk: {robot_dir / entry}")

    model_rel = robot_json.get("model")
    if not model_rel:
        raise SceneBuildError(f"{robot_dir / 'robot.json'} missing 'model'")
    _validate_relative(model_rel)
    model_path = robot_dir / model_rel
    if not model_path.is_file():
        raise SceneBuildError(f"model missing on disk: {model_path}")

    resolved_scene_id = scene_id or default_scene_id()
    if resolved_scene_id:
        model_path = Path(_resolve_scene_path(resolved_scene_id))

    return {
        "robot_id": robot_id,
        "scene_id": resolved_scene_id,
        "model_path": str(model_path),
        "assets": [str(robot_dir / e) for e in assets],
        "controlled_actuators": list(robot_json.get("controlledActuators", [])),
    }


if __name__ == "__main__":
    scene = build_scene()
    print(f"robot_id: {scene['robot_id']}")
    print(f"scene_id: {scene['scene_id']}")
    print(f"model_path: {scene['model_path']}")
    print(f"assets: {len(scene['assets'])} files")
    print(f"controlled_actuators: {scene['controlled_actuators']}")
