"""The arm host's code (arm_host/, run on the robot's Raspberry Pi) tested on the laptop: stdlib only, no hardware."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
