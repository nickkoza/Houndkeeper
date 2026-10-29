# Real robot

What runs on and around the real Go2, as opposed to the sim: bringing it up, calibrating it, and checking it.

bringup/      first boot (first-boot.md) and the robot's network with no way out (robot-network.md): dog-first-boot [MTv]
calibration/  a procedure for each estimated parameter (README.md, day one in order); measure.py reads a recording -> params/robot.yaml (source: measured)
night.sh      a find-and-flag night on the dog alone, one command from the laptop ([jFy])
tests/        the cockpit deadman (silence stops the robot, [Mj2]), the real night's scoring and the rehearsal on recorded data (tests/README.md)

## The operating area is not a safety barrier

The operating area ([JVr], drawn on the cockpit's Area page) keeps him to the rooms you choose
only while he knows where he is. It is not a safety barrier: keep stairs, drops and anything dangerous behind a
physical barrier, such as a gate, before he runs. On the robot a run does not start until an area is set.
