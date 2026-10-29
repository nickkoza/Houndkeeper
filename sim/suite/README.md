# Sim suite

Named sim scenarios, run the same way every time, with results kept in `history.jsonl`, so we can tell whether a change helped.

    ./run list                 scenarios + history
    ./run grasp --trials 40    grasp bench (the tight loop for the arm)
    ./run slice                full vertical slice (sweep, detect, plan stops, look, pick, bin)
    ./run walk | reach | power
    ./run tip                  tip safety: CoM margin over the reach (0 / 0.5 / 1.5 kg), fastest safe arm speed, measured tipping point
    ./run history grasp

The viewer is on by default; add `--headless` to turn it off. The viewer window is view-only: mouse drags push bodies.
Each record holds the args and a fingerprint of the sim code (sim/arm/*.py), so results line up with the code that produced them.
Compare grasp runs at the same `--seed`.

Tip safety (`./run tip`) writes `sim/arm/tip_envelope.json`: the safe reach per posture and payload, and the safe joint speed.
The arm code enforces a global joint speed limit (`Sim.ARM_VMAX`), set from this scenario.

Planned additions:
- soft sock (MuJoCo flexcomp cloth) and rope/string (MuJoCo cable)
- varied floor textures under the items: tile, carpet, rug, wood, and edges between them
- items near furniture and walls; lighting changes
