"""Unitree's Go2 CAD in the robot's frame, from the cache tools/go2_step_cache.py writes ([3JA] [I9y]).

The STEP is in millimetres with X across the body (left +), Y along it (head at low Y), Z up; its thigh joints lie on
Z = 0 at Y = 108 (front) and Y = 494 (rear), 386 mm apart, as the MuJoCo Menagerie model's hips are (2 x 193.4 mm),
so the body centre is (0, 301, 0). The robot frame used everywhere here (models/export, the sims): X forward, Y left,
Z up, origin at the body centre: X = 301 - Y_step, Y = X_step, Z = Z_step (a proper rotation).
Nothing here is committed geometry: the cache lives in models/go2/_src/ (Unitree's licence is not stated)."""
from functools import lru_cache
from pathlib import Path
import json
import numpy as np

R = Path(__file__).resolve().parents[2]
CACHE = R / "models/go2/_src/go2_cache"
BODY_CENTRE_STEP_Y = 301.0                 # midway between the thigh axes (Y 108 and 494 in the STEP)

# What the solids are (ids from the cache's index.json, read by hand from their size, place and holes, 4 Oct 2026)
DOCK = (214, 215, 216, 168, 169, 9, 33, 34, 35, 7, 8)   # the EDU expansion dock: housing, top rails, rear bracket + 2 M3 screws, flex, front connector
TOP_COVER = 6                              # the back's top cover (holes for the dock)
BODY_SHELL = 164
HEAD = 27
BATTERY = 29                               # the pack, a box on the left side (Y +52..+97), latches 30/31
BATTERY_PORT = 32                          # small block on the battery's outer face: its button/charge port (est.)
HEAD_CAMERA = 14                           # the front camera's lens, on the face
LIDAR = (24, 26, 28)                       # the L1 under the chin


def available():
    return (CACHE / "solids.npz").exists()


def step_to_robot(P):
    P = np.asarray(P, float)
    return np.stack([BODY_CENTRE_STEP_Y - P[..., 1], P[..., 0], P[..., 2]], axis=-1)


@lru_cache(maxsize=1)
def _load():
    z = np.load(CACHE / "solids.npz"); idx = json.loads((CACHE / "index.json").read_text(encoding="utf-8"))["solids"]
    V = step_to_robot(z["V"]); F = z["F"][:, [0, 2, 1]]       # the frame change is a rotation, but keep normals out
    return V, F, z["SID"], idx


def index():
    return _load()[3]


def mesh(include=None, exclude=()):
    """the Go2 (robot frame, mm) as one trimesh: solids in `include` (all if None), minus `exclude`"""
    import trimesh
    V, F, S, _ = _load()
    keep = np.ones(len(F), bool) if include is None else np.isin(S, list(include))
    if exclude: keep &= ~np.isin(S, list(exclude))
    Fk = F[keep]; u, inv = np.unique(Fk, return_inverse=True)
    return trimesh.Trimesh(V[u], inv.reshape(-1, 3), process=False)


def holes(sid, rmax=8.0, vertical=True):
    """a solid's cylindrical faces as (X, Y, Z, diameter, height, is_hole), robot frame"""
    out = []
    for c in index()[sid]["cylinders"]:
        if vertical and abs(c["d"][2]) < 0.99: continue
        if c["r"] > rmax: continue
        X, Y, Z = step_to_robot(c["c"])
        out.append((round(float(X), 1), round(float(Y), 1), round(float(Z), 1), round(2 * c["r"], 2), c["ext"][2], c["inner"]))
    return out


def top_z(m, xs, ys, z0=400.0):
    """height of the highest surface of mesh m above each (x, y): the back's top contour"""
    O = np.array([[x, y, z0] for x in xs for y in ys], float); D = np.tile([0, 0, -1.0], (len(O), 1))
    loc, ir, _ = m.ray.intersects_location(O, D, multiple_hits=True)
    Z = np.full(len(O), np.nan)
    for i, z in zip(ir, loc[:, 2]):
        if np.isnan(Z[i]) or z > Z[i]: Z[i] = z
    return Z.reshape(len(xs), len(ys))
