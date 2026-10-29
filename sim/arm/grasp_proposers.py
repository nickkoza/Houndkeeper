"""Grasp proposers behind one interface (sim-grasp [WuR]): given the look-closer's result (`look`: its fused find,
with the looks' item masks, stereo heights and camera poses), a proposer returns its grasps best first in DimOS's own
grasp message, with each grasp's grip width and how far the pick may adapt it. The pick runs the first the arm can
reach, whatever proposed it. Chosen by HK_GRASP:

  ours       Houndkeeper's grasp, from the look as it is: the grip options measured on the item's footprint (the
             default; no point cloud)
  dimos      DimOS's HeuristicGraspModule (waymark [dgF]), through the point-cloud adapter
  graspgenx  DimOS's GraspGenX runtime (waymark [uVy]), through the point-cloud adapter, served over a pipe

Poses follow DimOS's grasp convention (its HeuristicGraspModule, and GraspGenX through its tool-point transform):
the position is the grasp centre, midway between the jaws; z points along the approach, into the item; y is the way
the jaws close."""
import math, os, sys
from dataclasses import dataclass
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src")); from houndkeeper import workspace   # noqa: E402  (decision [cy4])

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.std_msgs.Header import Header

ITEM_MIN_H = 0.004        # m: a point this far off the floor is the item's, not floor at its edge
DIMOS_LEAN = float(os.environ.get("HK_DIMOS_LEAN", "2"))  # deg: DimOS's grasp is straight down (2: slack for the IK);
                                                         # 30: its point and jaw with our lean, to split the two ([dgF])
GGX_PROJECT = str(workspace.dimos() / "native/python/graspgenx")     # DimOS's isolated GraspGenX runtime project
GGX_VENV = os.path.expanduser("~/.cache/dimos/isolated-python/graspgenx-hk/.venv")
GGX_POINTS = 2048         # the cloud GraspGenX is given, thinned evenly at random to this
GGX_TRY = 30              # the best-scored grasps offered, of the 100 it returns
GGX_TILT = float(os.environ.get("HK_GGX_TILT", "0"))  # deg a grasp may be turned toward upright when its exact pose is out of
                                                      # reach (grip point and jaw kept); 0: exactly as GraspGenX gives it ([uVy])
GRIP_LIFT = os.environ.get("HK_GRIP_LIFT", "0") == "1"   # sim-grasp [GU5]: tall items gripped higher than the floor
GRIP_LIFT_TALL_M = 0.06   # an item this tall (its stereo points' 95th percentile) or taller is gripped higher ([GU5])
GRIP_LIFT_FRAC, GRIP_LIFT_MAX_M = 0.35, 0.04   # the fingertips at this fraction of its height, at most this high
GGX_RAISE_M = 0.015       # GraspGenX's cloud has no floor: its grasps may be raised this much to keep the tips off it


@dataclass(frozen=True)
class Allowance:
    """How far the pick may adapt a proposed grasp to reach it. around "vertical": the gripper may lean up to lean_deg
    from straight down, closing along the grasp's jaw direction (the pose's own tilt is not kept); "pose": the pose's
    orientation, turned toward upright by up to lean_deg. height "tips": set so the fingertips sit tip_m above the
    floor; "raise": the pose's height, raised (by at most raise_max_m) only to keep the tips tip_m above the floor"""
    lean_deg: float
    around: str = "vertical"
    height: str = "tips"
    tip_m: float = 0.003
    raise_max_m: float = math.inf


@dataclass
class Proposal:
    """a proposer's grasps (DimOS's message, best first), with each one's grip width (m) and allowance"""
    grasps: GraspCandidateArray
    widths_m: list
    allowances: list


def tcp_pose(p, R):
    """DimOS Pose from a grasp centre and a grasp-frame rotation (columns x, y closing, z approach)"""
    x, y, z, w = Rotation.from_matrix(np.asarray(R, float)).as_quat()
    return Pose(Vector3(float(p[0]), float(p[1]), float(p[2])), Quaternion(float(x), float(y), float(z), float(w)))


def pose_arrays(pose):
    """(grasp centre, grasp-frame rotation) of a DimOS Pose"""
    q = pose.orientation
    return (np.array([pose.position.x, pose.position.y, pose.position.z]),
            Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix())


def link7_from_tcp(R):
    """the wrist's (link 7) orientation for a grasp-frame rotation: its y closes the jaws, its -z is the approach"""
    return np.column_stack([-R[:, 0], R[:, 1], -R[:, 2]])


def _straight_down(jaw_xy):
    """a grasp frame pointing straight down, closing along jaw_xy"""
    y = np.array([jaw_xy[0], jaw_xy[1], 0.0]); y /= np.linalg.norm(y); z = np.array([0.0, 0.0, -1.0])
    return np.column_stack([np.cross(y, z), y, z])


def _array(poses, scores):
    return GraspCandidateArray(Header(0.0, "world"), [GraspCandidate(tcp_pose(p, R), float(s)) for (p, R), s in zip(poses, scores)])


class HoundkeeperGrasps:
    """our grasp from the look as it is: the item's grip options measured on its footprint (width, jaw direction,
    offset from its middle; narrowest first, a part where the whole is too wide), each straight down at the option,
    to lean up to MAX_LEAN (THIN_LEAN for thin items) with the fingertips just above the floor"""
    name = "ours"

    def propose(self, look):
        import pick
        o = np.array([look["xy"][0], look["xy"][1], 0.0])
        opts = look.get("grip_options", [(look["grip_width_cm"] / 100, look["short_axis"], np.zeros(2))])
        opts = pick.real_grips(opts)[:12]
        lift = None                                                           # [GU5]: a tall item gripped only at its foot turns
        if GRIP_LIFT:                                                         # in the jaws (a shoe's flared sole, 30 Sep)
            pts = item_cloud(look); tall = None if pts is None else float(np.percentile(pts[:, 2], 95))
            if tall is not None and tall >= GRIP_LIFT_TALL_M: lift = min(GRIP_LIFT_FRAC * tall, GRIP_LIFT_MAX_M)
        poses, widths, allow = [], [], []
        for w_, jaw, off in opts:
            thin = w_ < pick.THIN
            poses.append((o + [off[0], off[1], 0.04], _straight_down(jaw))); widths.append(float(w_))
            tip = pick.THIN_TIP if thin else (max(pick.TIP_CLEAR, lift) if lift else pick.TIP_CLEAR)
            allow.append(Allowance(min(pick.MAX_LEAN, pick.THIN_LEAN) if thin else pick.MAX_LEAN, tip_m=tip))
        return Proposal(_array(poses, [1.0 - k / max(len(poses), 1) for k in range(len(poses))]), widths, allow)


def item_cloud(look):
    """the point-cloud adapter's cloud: the pair's points in the look's item mask standing off the floor (world, m);
    None without the pair or with under 30 points ([dgF], [uVy])"""
    cv = look.get("_cv"); mk = look.get("mask")
    if cv is None or mk is None or cv.get("stereo_P") is None: return None
    h = cv["stereo_h"]; sel = mk & np.isfinite(h) & (h > ITEM_MIN_H) & ~cv["robot"]
    return cv["stereo_P"][sel].astype(np.float32) if sel.sum() >= 30 else None


class DimosHeuristicGrasps:
    """DimOS's HeuristicGraspModule on the look's cloud ([dgF]): one grasp at the cloud's middle, closing across its
    narrow side, straight down (DIMOS_LEAN of slack), at its mid-height, raised only to keep the tips off the floor"""
    name = "dimos"

    def propose(self, look):
        import pick
        pts = item_cloud(look)
        if pts is None: return "no point cloud for the DimOS grasp"
        from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule as H
        from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
        g = H.propose_grasps(H, PointCloud2.from_numpy(pts, frame_id="world", timestamp=1.0))
        w_ = look["grip_width_cm"] / 100
        return Proposal(g, [w_] * len(g.candidates),
                        [Allowance(DIMOS_LEAN, height="raise", tip_m=pick.THIN_TIP if w_ < pick.THIN else pick.TIP_CLEAR)] * len(g.candidates))


class GraspGenXClient:
    """GraspGenX served by graspgenx_server.py in DimOS's isolated environment, over a pipe ([uVy])"""
    def __init__(self):
        import pickle, subprocess                                            # a pipe to our own child process
        env = dict(os.environ); [env.pop(k, None) for k in ("VIRTUAL_ENV", "UV_PYTHON")]
        env.update(UV_PROJECT_ENVIRONMENT=GGX_VENV, PYTHONPATH=GGX_PROJECT); self._pk = pickle
        if env.get("HK_GGX_DEVICE") == "cpu": env["CUDA_VISIBLE_DEVICES"] = ""   # its process only: the sim keeps the GPU
        self.p = subprocess.Popen(["uv", "run", "--frozen", "--with-editable", str(Path(GGX_PROJECT).parents[2]), "python",
                                   str(Path(__file__).with_name("graspgenx_server.py"))], cwd=GGX_PROJECT, env=env,
                                  stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        self.info = pickle.load(self.p.stdout)
    def ask(self, pts): self._pk.dump(pts, self.p.stdin); self.p.stdin.flush(); return self._pk.load(self.p.stdout)


class GraspGenXGrasps:
    """DimOS's GraspGenX on the look's cloud ([uVy]), thinned to GGX_POINTS: its best GGX_TRY grasps that approach
    from above (the cloud has no floor, so some come from under it), each as given (turned up to GGX_TILT toward
    upright if out of reach), raised by at most GGX_RAISE_M to keep the tips off the floor"""
    name = "graspgenx"
    _client = None

    def propose(self, look):
        import pick
        pts = item_cloud(look)
        if pts is None: return "no point cloud for GraspGenX"
        if len(pts) > GGX_POINTS:                                           # the full-resolution cloud runs to tens of thousands
            pts = pts[np.random.default_rng(0).choice(len(pts), GGX_POINTS, replace=False)]   # of points; memory grows with them
        if GraspGenXGrasps._client is None: GraspGenXGrasps._client = GraspGenXClient()
        tcp, scores = GraspGenXGrasps._client.ask(pts)                       # grasp-frame poses, best first
        keep = [k for k in range(len(scores)) if tcp[k][2, 2] < -0.1][:GGX_TRY]
        if not keep: return "no GraspGenX grasp from above"
        w_ = look["grip_width_cm"] / 100
        return Proposal(_array([(tcp[k][:3, 3], tcp[k][:3, :3]) for k in keep], [scores[k] for k in keep]), [w_] * len(keep),
                        [Allowance(GGX_TILT, around="pose", height="raise", tip_m=pick.TIP_CLEAR, raise_max_m=GGX_RAISE_M)] * len(keep))


PROPOSERS = {c.name: c for c in (HoundkeeperGrasps, DimosHeuristicGrasps, GraspGenXGrasps)}


def proposer(name=None):
    """the configured proposer (HK_GRASP, default ours)"""
    return PROPOSERS[name or os.environ.get("HK_GRASP", "ours")]()
