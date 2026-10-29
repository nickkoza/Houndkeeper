"""Where the sibling checkouts are (decision [cy4]): the workspace is the directory holding this repository, unless
HK_WORKSPACE names another; DimOS is HK_DIMOS when set, else the workspace's dimos."""
import os
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]


def workspace() -> Path:
    return Path(os.environ.get("HK_WORKSPACE") or REPO.parent)


def sibling(name: str) -> Path:
    """a checkout or output directory beside this repository: dimos-ext, dimos-twin, maker-arm-sdk, build"""
    return workspace() / name


def dimos() -> Path:
    return Path(os.environ.get("HK_DIMOS") or sibling("dimos"))
