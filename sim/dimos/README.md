# Houndkeeper on DimOS's own sim

The tidy loop end to end in DimOS's MuJoCo sim ([YWo]), with the Maker Arm mounted on the robot ([RJi]).

Needs the DimOS fork changes in `dimos_fork_changes.patch` (model hook, legs-only walking policy, postures, arm through
DimOS's sim-arm shared memory, arm camera stream, CPU walking policy, sim truth file). Apply in the DimOS checkout beside this repo (`../dimos` from its root)
with `git apply` if the fork was reset.

| Script | What it does |
|---|---|
| `tidy_run.sh [items json] [start]` | Blueprint `unitree-go2 mcp-server houndkeeper.tidy`, viewer on, real time; calls `tidy_start`, polls `tidy_status`, then scores against the sim truth (`score_tidy.py`). Output under `data/runs/dimos_tidy/`. Set `HK_PICK_POLICY=<checkpoint>` for the learned pick. Runs in the PoC house by default (`ROOM=nights_s70_n1`: the house and a night's floor, no built-in clutter; `make_nights.py --seed 70 --nights 1 --per-night 0` rebuilds it), with library items placed by `house_items.py` (`N_ITEMS`, `ITEM_SEED`); `ROOM=office1` for DimOS's stock scene. Our HUD draws on DimOS's viewer (`DIMOS_SIM_VIEWER_HOOK`, set by `env.py`). |
| `arm_bringup.py` | Tight loop: lie down, drive the arm through DimOS's adapter, read the arm camera, stand, walk. |
| `pick_bringup.py <item> <flag error m> <out> <start>` | Tight loop: one item beside the robot, look closer and pick through DimOS; `HK_PICK_POLICY` for the learned pick. |
| `env.py` | The environment both sides read (hook, arm joints, camera). |

Run one DimOS job at a time: the RTX 2070's 8 GB doesn't hold DimOS, the detectors and training together, and DimOS's
sim needs the CPU to keep real time (the tidy module's controllers run on the wall clock, as on the real robot).
