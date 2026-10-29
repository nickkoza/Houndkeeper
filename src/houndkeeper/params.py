"""Robot parameters, read from params/robot.yaml: the one place a physical number lives, with its source."""
from functools import lru_cache
from pathlib import Path
import yaml

FILE = Path(__file__).resolve().parents[2] / "params" / "robot.yaml"


@lru_cache(maxsize=1)
def _all():
    return yaml.safe_load(FILE.read_text())


def get(path):
    """get("maker_arm.mount_x_m") -> the value. An unknown (null) value raises, so no one silently uses a guess."""
    node = _all()
    for key in path.split("."):
        node = node[key]
    if node["value"] is None:
        raise ValueError(f"{path} is unknown in {FILE} ({node.get('note', 'no note')}); measure it first")
    return node["value"]
