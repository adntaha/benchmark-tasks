"""totekit: an attached-lid tote (a flip-top container: a tub with two lids hinged on its long sides, whose free edges
meet along a stepped seam, each lid wider in one half and narrower in the other), for yam_arena task files.

    import pathlib, sys
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    import totekit as K

The tote is one articulation: the tub is the root (a free body on the table) and each lid is a link on a revolute joint
about the tub's x axis (its long axis), at the top outer edge of a long wall. Every part is made of cuboid colliders:
the lids' overlap (in each half of the seam the wider lid has a raised flange over the other lid's low tongue) is real boxes,
so the two lids collide with each other where they meet; whether a lid closed first blocks the second is the geometry's
doing (measured, see the README). There is no snap or latch: a lid shut on the rim stays there by its weight.

Joint angles (rad), measured on each lid as the angle it is open by (theta, 0 closed and flat on the rim, 90 standing
up, 180 flat outward, THETA_OPEN resting open flat against the hinge stop); the simulator's own joint angle is -theta for the
+y lid and +theta for the -y lid. :func:`lid_angles` returns thetas.

``on_step`` and ``on_reset`` register callbacks ``fn(env)``. Pure Python at import time; Isaac Sim is reached only when
the scene is built. The hooks are installed through ``sys.meta_path`` the way yam_arena's own modules need (the task
file is read before the simulator starts).
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena.task_file import Box, ObjectState

__all__ = ["Tote", "ToteState", "finger_points", "install", "lid_angles", "lid_point", "on_reset", "on_step"]

# -- the Akro-Mils 39085 attached lid container, outside dimensions as listed by retailers (m) -------------------------
LENGTH, WIDTH, HEIGHT = 0.5461, 0.381, 0.2286     # 21.5 x 15 x 9 in; the maker's own page is reported to say 15.25 in wide
# -- ESTIMATED geometry and masses (not from a catalogue): see README -----------------------------------------------------
WALL = 0.004                    # wall thickness
FLOOR = 0.008                   # floor thickness
LID_T = 0.004                   # lid plate thickness
HINGE_OUT = 0.004               # the hinge axis sits this far outside the wall's outer face, so a hanging lid clears it
LID_LEN = LENGTH - 0.008        # the lid's length along the hinge
CENTRE_GAP = 0.010              # each lid's low plate ends this far from the tote's centre line (low tongues cross it)
OVERLAP = 0.008                 # a raised finger reaches this far past the centre line over the other lid's low tongue
LIP_LEVEL = 0.016               # the raised fingers' underside is this far above the plate's lower face (a 12 mm air gap)
RISER_T = 0.004                 # the riser carrying a raised finger
TONGUE_STOP = 0.002             # a low tongue stops this short of the other lid's riser
SEAM_OFFSET = 0.0175            # ESTIMATE (about 9 % of a lid's width): the seam's centre line is this far toward -y in the
                                # -x half and toward +y in the +x half, with a 45 degree jog between
JOG_HALF = 0.0085               # half the jog's length along x (45 degrees: the lids' edges move 2 x JOG_HALF)
JOG_STEPS = 4                   # the diagonal is built as this many small steps
BLOCK = (0.020, 0.025, 0.008)   # raised seal block at each lid's seam end (x, u, height); the hole is a gap in it
TOTE_MASS, LID_MASS = 1.9, 0.4  # kg
LID_DAMPING = 0.15              # N m s / rad: viscous hinge friction
THETA_OPEN = math.radians(180.0)   # ESTIMATE: the hinge stop; a lid opens this far and rests against it, flat out beside the wall

# -- product-photo details, all ESTIMATED from a description of the photo (see README) ----------------------------------
A_TOP = 0.006                   # the wall's outer face is this far in from the rim band's outer face (the band stands proud)
A_BASE = 0.025                  # the wall's outer face at the base is this far in from the rim outline (nestable taper)
RIM_H, RIM_T = 0.035, 0.012     # rim band: height and horizontal thickness
ZT = HEIGHT - RIM_H             # the band's underside; the tapered wall runs from z 0 to here
PHI = math.atan((A_BASE - A_TOP) / ZT)      # wall lean from vertical (about 5.6 degrees)
FOOT_H, FOOT_W = 0.006, 0.012   # recessed foot: the tub stands on a ring this tall and wide, the floor sits above it
SLOT_H, SLOT_GAP = 0.020, 0.004     # slots under the rim band: height, and the wall left between slot top and band
SLOT_W_LONG, SLOT_W_END = 0.060, 0.050
RIB_W, RIB_PROUD = 0.014, 0.003     # visual ribs between the recessed panels
LUG = (0.050, 0.030, 0.007)         # lid stacking lug (x, u, height) at each outer corner, u from the hinge side
LUG_U0, LUG_X0 = 0.016, 0.200       # lug starts this far from the hinge axis and this far from the lid's middle (|x|)
HOLE = 0.006                        # square hole through each seal block (a gap between boxes, not round)
KNUCKLES, KNUCKLE_LEN, KNUCKLE_R = 7, 0.040, 0.006     # visual hinge knuckles per side
BOSS_R, BOSS_H, BOSS_U, BOSS_X = 0.012, 0.003, 0.080, 0.100   # two round bosses on each lid's top (visual only)


def inset(z: float) -> float:
    """How far the tapered wall's outer face is in from the rim outline at height z (z in 0..ZT)."""
    return A_BASE + (A_TOP - A_BASE) * z / ZT


def wall_piece(kind: str, sgn: int, z0: float, z1: float, c0: float, c1: float, thick: float, offset: float):
    """A tilted box on the tapered wall: ``kind`` 'long' (the wall at y = sgn * ..., c along x) or 'end' (at x = sgn * ..., c
    along y), between wall heights z0 and z1 measured on the outer face, c0..c1 along the wall. ``offset`` is the box's
    distance from the outer face along the outward normal (negative = inside the wall) of its middle plane. Returns
    (centre, size, quat_wxyz)."""
    zm = (z0 + z1) / 2
    half = (WIDTH if kind == "long" else LENGTH) / 2 - inset(zm)      # outer face's horizontal half-extent at zm
    h = half + offset * math.cos(PHI)
    z = zm - offset * math.sin(PHI)
    cc, ln, hz = (c0 + c1) / 2, c1 - c0, (z1 - z0) / math.cos(PHI)
    if kind == "long":
        al = -sgn * PHI / 2
        return (cc, sgn * h, z), (ln, thick, hz), (math.cos(al), math.sin(al), 0.0, 0.0)
    be = sgn * PHI / 2
    return (sgn * h, cc, z), (thick, ln, hz), (math.cos(be), 0.0, math.sin(be), 0.0)


def _segments(lo, hi, gaps):
    out, a = [], lo
    for g0, g1 in sorted(gaps):
        if g0 > a:
            out.append((a, g0))
        a = g1
    if hi > a:
        out.append((a, hi))
    return out


def panel_ribs(kind: str):
    """Rib and slot centres along a wall: 5 panels on a long wall, 2 on an end wall."""
    zm = ZT / 2
    n = 5 if kind == "long" else 2
    half = (LENGTH if kind == "long" else WIDTH) / 2 - inset(zm) - 0.016
    ribs = [-half + 2 * half * i / n for i in range(n + 1)]
    return ribs, [(a + b) / 2 for a, b in zip(ribs, ribs[1:])]


def tub_parts() -> list:
    """Every part of the tub as (name, centre, size, quat_wxyz or None, collide). Collision: tapered walls (with real slot
    openings under the rim band), rim band, foot ring, floor. Visual only: the ribs between the recessed panels."""
    parts = []
    zs0, zs1 = ZT - SLOT_GAP - SLOT_H, ZT - SLOT_GAP
    for kind in ("long", "end"):
        for sgn in (1, -1):
            tag = f"{kind}_{'pn'[sgn < 0]}"
            half = lambda z: (LENGTH if kind == "long" else WIDTH) / 2 - inset(z) - (0.0 if kind == "long" else 0.002)
            _, centres = panel_ribs(kind)
            sw = SLOT_W_LONG if kind == "long" else SLOT_W_END
            slots = [(c - sw / 2, c + sw / 2) for c in (centres[1], centres[-2]) ] if kind == "long" else [(c - sw / 2, c + sw / 2) for c in centres]
            for nm, (z0, z1, gaps) in {"lo": (0.0, zs0, []), "mid": (zs0, zs1, slots), "top": (zs1, ZT, [])}.items():
                zm = (z0 + z1) / 2
                for si, (c0, c1) in enumerate(_segments(-half(zm), half(zm), gaps)):
                    ctr, size, q = wall_piece(kind, sgn, z0, z1, c0, c1, WALL, -WALL / 2)
                    parts.append((f"wall_{tag}_{nm}{si}", ctr, size, q, True))
            ribs, _ = panel_ribs(kind)
            for ri, c in enumerate(ribs):
                if kind == "end" and ri in (0, len(ribs) - 1):
                    continue                                         # the corner ribs belong to the long walls
                ctr, size, q = wall_piece(kind, sgn, 0.010, ZT - 0.001, c - RIB_W / 2, c + RIB_W / 2, RIB_PROUD, RIB_PROUD / 2)
                parts.append((f"rib_{tag}_{ri}", ctr, size, q, False))
    zb = HEIGHT - RIM_H / 2                                          # rim band: a ring from z ZT to HEIGHT, outline = the catalogue size
    for sgn, tag in ((1, "pos"), (-1, "neg")):
        parts.append((f"rim_{tag}", (0.0, sgn * (WIDTH / 2 - RIM_T / 2), zb), (LENGTH, RIM_T, RIM_H), None, True))
        parts.append((f"rim_end_{tag}", (sgn * (LENGTH / 2 - RIM_T / 2), 0.0, zb), (RIM_T, WIDTH - 2 * RIM_T, RIM_H), None, True))
    lx, wy = LENGTH - 2 * A_BASE, WIDTH - 2 * A_BASE                 # foot ring (outline = the base footprint) and the floor above it
    for sgn, tag in ((1, "pos"), (-1, "neg")):
        parts.append((f"foot_{tag}", (0.0, sgn * (wy / 2 - FOOT_W / 2), FOOT_H / 2), (lx, FOOT_W, FOOT_H), None, True))
        parts.append((f"foot_end_{tag}", (sgn * (lx / 2 - FOOT_W / 2), 0.0, FOOT_H / 2), (FOOT_W, wy - 2 * FOOT_W, FOOT_H), None, True))
    parts.append(("floor", (0.0, 0.0, FOOT_H + FLOOR / 2), (lx, wy, FLOOR), None, True))
    return parts


HINGE_Y = WIDTH / 2 + HINGE_OUT                 # |y| of the hinge axes
PLATE_LEN = HINGE_Y - CENTRE_GAP                # hinge axis to the low plate's edge (the riser's face)
FINGER_REACH = HINGE_Y + OVERLAP                # (nominal, seam on the centre line) hinge axis to a raised flange's tip
TONGUE_REACH = HINGE_Y + CENTRE_GAP - TONGUE_STOP   # (nominal) hinge axis to a low tongue's tip


def centre_y(x: float) -> float:
    """The seam's centre line (world/tote-frame y) at tote-frame ``x`` outside the jog: -SEAM_OFFSET for x < 0, else +."""
    return -SEAM_OFFSET if x < 0 else SEAM_OFFSET


WIDE = {-1: +1, +1: -1}      # zone (sign of x) -> the lid side that is wider there: the +y lid in the -x half, the -y lid in the +x half


def lid_zones(side: int) -> list[dict]:
    """The lid at ``side``'s free-edge profile along x: zones with x0, x1, the low plate's end u_end (distance from the
    hinge axis), and for the wider lid a raised flange: u_flange_tip. The jog is a staircase of plain low steps."""
    def ends(zone):                                         # zone: -1 (the -x half) or +1
        yc = -SEAM_OFFSET if zone < 0 else SEAM_OFFSET
        if WIDE[zone] == side:                              # wider here: low plate to its edge, a riser there, a raised flange over the other lid
            return HINGE_Y - side * yc - CENTRE_GAP, HINGE_Y - side * yc + OVERLAP
        return HINGE_Y - side * yc + CENTRE_GAP - TONGUE_STOP, None   # narrower: a low tongue under the other lid's flange

    (u1, f1), (u3, f3) = ends(-1), ends(+1)
    zones = [dict(x0=-LID_LEN / 2, x1=-JOG_HALF, u_end=u1, flange=f1), dict(x0=JOG_HALF, x1=LID_LEN / 2, u_end=u3, flange=f3)]
    for k in range(JOG_STEPS):
        a, b = -JOG_HALF + 2 * JOG_HALF * k / JOG_STEPS, -JOG_HALF + 2 * JOG_HALF * (k + 1) / JOG_STEPS
        u = u1 + (u3 - u1) * (k + 0.5) / JOG_STEPS
        zones.append(dict(x0=a, x1=b, u_end=u, flange=None))
    return zones


@dataclass(frozen=True)
class Tote(Box):
    """The tote. ``pos`` is the middle of its footprint on the table (z the table top), ``quat`` (xyzw) its
    orientation; ``mass`` is the tub's (the lids add 2 x LID_MASS)."""

    quat: tuple = (0.0, 0.0, 0.0, 1.0)
    colour: tuple = (0.14, 0.34, 0.70)          # mid blue, one solid colour for tub and lids
    lid_colour: tuple = (0.14, 0.34, 0.70)
    friction: tuple = (0.45, 0.35)
    meta: dict = field(default_factory=dict, compare=False, hash=False)

    def _check(self) -> None:
        if not self.name.isidentifier():
            raise ValueError(f"object name {self.name!r} must be a Python identifier")
        if self.mass is None or self.mass <= 0:
            raise ValueError(f"{self.name}: a Tote must be loose (give it a mass)")


@dataclass(frozen=True)
class ToteState(ObjectState):
    """The tote as the simulator has it; ``joints`` maps ``lid_pos``/``lid_neg`` to their opening angles theta (rad),
    ``joint_vel`` to their angular speeds (rad/s)."""

    joints: dict = field(default_factory=dict)
    joint_vel: dict = field(default_factory=dict)


def _rotation(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


GAUGE_X = {+1: -(LID_LEN / 2 + JOG_HALF) / 2, -1: (LID_LEN / 2 + JOG_HALF) / 2}   # middle of the straight seam section where each lid is the wider one


def gauge_u(side: int) -> float:
    """Distance from the hinge axis to the lid's free (seam-side) plate edge at its gauge point GAUGE_X[side]."""
    return next(z["u_end"] for z in lid_zones(side) if z["x0"] <= GAUGE_X[side] <= z["x1"])


def edge_height(side: int, theta: float) -> float:
    """Height (m) of the lid's free plate edge (its lower corner) above the rim plane at its gauge point."""
    return gauge_u(side) * math.sin(theta)


def lid_point(side: int, theta: float, s: float, offset: float = 0.0, x: float = 0.0) -> np.ndarray:
    """A point on a lid's wall-facing face (its underside when open past 90 degrees), in the tote's frame (origin the
    middle of its footprint, z up): ``s`` metres out from the hinge axis along the lid, ``offset`` metres off that face
    (positive is away from the lid, on the side of the wall/ground), at ``x`` along the hinge."""
    d = np.array([-side * math.cos(theta), math.sin(theta)])            # (y, z) along the lid from the hinge
    n = np.array([side * math.sin(theta), math.cos(theta)])             # normal of the face turned toward the wall/ground
    yz = np.array([side * HINGE_Y, HEIGHT]) + s * d + offset * n
    return np.array([x, yz[0], yz[1]])


# -- authoring helpers --------------------------------------------------------------------------------------------------

def _material(sim_utils, sf, df):
    try:
        return sim_utils.RigidBodyMaterialCfg(static_friction=sf, dynamic_friction=df, friction_combine_mode="average")
    except TypeError:
        return sim_utils.RigidBodyMaterialCfg(static_friction=sf, dynamic_friction=df)


class _Author:
    def __init__(self, prim_path, translation, orientation):
        import isaaclab.sim as sim_utils
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, UsdGeom

        self.sim_utils, self.Gf, self.UsdGeom = sim_utils, Gf, UsdGeom
        self.stage = get_current_stage()
        self.path = prim_path
        double = UsdGeom.XformOp.PrecisionDouble
        root = UsdGeom.Xform.Define(self.stage, prim_path)
        root.AddTranslateOp(precision=double).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
        x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
        root.AddOrientOp(precision=double).Set(Gf.Quatd(w, x, y, z))
        self.root = root.GetPrim()
        self.looks, self.mats = {}, {}

    def look(self, color, rough=0.6) -> str:
        from yam_arena.station import linear_rgb

        key = (tuple(color), rough)
        if key not in self.looks:
            path = f"{self.path}/Looks/v{len(self.looks)}"
            vis = self.sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=rough)
            vis.func(path, vis)
            self.looks[key] = path
        return self.looks[key]

    def mat(self, key, sf, df) -> str:
        if key not in self.mats:
            path = f"{self.path}/Looks/phys_{key}"
            pm = _material(self.sim_utils, sf, df)
            pm.func(path, pm)
            self.mats[key] = path
        return self.mats[key]

    def body(self, path, pos, mass, ang_damp=0.1):
        from pxr import PhysxSchema, UsdPhysics

        xf = self.UsdGeom.Xform.Define(self.stage, path)
        if path != self.path and pos is not None:
            xf.AddTranslateOp(precision=self.UsdGeom.XformOp.PrecisionDouble).Set(self.Gf.Vec3d(*pos))
            xf.AddOrientOp(precision=self.UsdGeom.XformOp.PrecisionDouble).Set(self.Gf.Quatd(1, 0, 0, 0))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(mass))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(0.3)
        rb.CreateLinearDampingAttr(0.05)
        rb.CreateAngularDampingAttr(ang_damp)
        return prim

    def box(self, path, centre, size, look, mat, quat_wxyz=None, collide=True):
        from isaaclab.sim.utils import bind_physics_material, bind_visual_material
        from pxr import PhysxSchema, UsdPhysics

        cube = self.UsdGeom.Cube.Define(self.stage, path)
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(self.Gf.Vec3d(*(float(v) for v in centre)))
        if quat_wxyz is not None:
            cube.AddOrientOp().Set(self.Gf.Quatf(*(float(v) for v in quat_wxyz)))
        cube.AddScaleOp().Set(self.Gf.Vec3f(*(float(v) for v in size)))
        prim = cube.GetPrim()
        bind_visual_material(str(prim.GetPath()), look, stage=self.stage)
        if not collide:
            return prim
        UsdPhysics.CollisionAPI.Apply(prim)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        cp.CreateContactOffsetAttr(0.0008)
        cp.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(prim.GetPath()), mat, stage=self.stage)
        return prim

    def cylinder(self, path, centre, radius, height, axis, look):
        """A visual-only cylinder (no collider)."""
        from isaaclab.sim.utils import bind_visual_material

        cyl = self.UsdGeom.Cylinder.Define(self.stage, path)
        cyl.CreateAxisAttr(axis)
        cyl.CreateRadiusAttr(float(radius))
        cyl.CreateHeightAttr(float(height))
        cyl.AddTranslateOp().Set(self.Gf.Vec3d(*(float(v) for v in centre)))
        bind_visual_material(str(cyl.GetPrim().GetPath()), look, stage=self.stage)
        return cyl.GetPrim()


# -- spawning ------------------------------------------------------------------------------------------------------------

_SPECS: dict = {}
SIDES = {"lid_pos": +1, "lid_neg": -1}       # joint name -> side (+1: hinged at +y)


def _spawn_tote(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    from pxr import Gf, PhysxSchema, Sdf, UsdPhysics

    spec: Tote = _SPECS[cfg.kit_name]
    a = _Author(prim_path, translation, orientation)
    grey, lid_grey = a.look(spec.colour), a.look(spec.lid_colour)
    m_tub, m_lid = a.mat("tub", *spec.friction), a.mat("lid", 0.4, 0.3)

    tub = f"{prim_path}/tub"
    a.body(tub, (0.0, 0.0, 0.0), spec.mass, ang_damp=0.05)
    for nm, centre, size, quat, collide in tub_parts():
        a.box(f"{tub}/{nm}", centre, size, grey, m_tub, quat_wxyz=quat, collide=collide)
    stage = a.stage
    for jname, side in SIDES.items():
        body = f"{prim_path}/{jname}"
        a.body(body, (0.0, side * HINGE_Y, HEIGHT), LID_MASS, ang_damp=0.05)
        # lid frame: origin on the hinge axis at the plate's lower face; the closed plate runs toward the centre line
        # (local y = -side * u) with u the distance from the hinge axis. Boxes are given as (u0, u1, w0, w1, x0, x1).
        def lidbox(tag, u0, u1, w0, w1, x0, x1, collide=True):
            a.box(f"{body}/{tag}", ((x0 + x1) / 2, -side * (u0 + u1) / 2, (w0 + w1) / 2), (x1 - x0, u1 - u0, w1 - w0), lid_grey, m_lid,
                  collide=collide)

        for zi, z in enumerate(lid_zones(side)):
            lidbox(f"edge_{zi}", 0.0, z["u_end"], 0.0, LID_T, z["x0"], z["x1"])
            if z["flange"] is not None:                         # the wider lid's raised flange on a riser, over the other's tongue
                lidbox(f"riser_{zi}", z["u_end"] - RISER_T, z["u_end"], 0.0, LIP_LEVEL + LID_T, z["x0"], z["x1"])
                lidbox(f"flange_{zi}", z["u_end"] - RISER_T, z["flange"], LIP_LEVEL, LIP_LEVEL + LID_T, z["x0"], z["x1"])
        for e in (1, -1):                                       # raised seal block at each seam end, with a square hole through it
            ua, ub = PLATE_LEN - 0.025 - BLOCK[1], PLATE_LEN - 0.025
            xa, xb = sorted((e * (LID_LEN / 2 - 0.030), e * (LID_LEN / 2 - 0.010)))
            xc, uc, h = (xa + xb) / 2, (ua + ub) / 2, HOLE / 2
            t, zt_ = f"block_{'ew'[e < 0]}", LID_T + BLOCK[2]
            lidbox(t + "_a", ua, ub, LID_T, zt_, xa, xc - h)
            lidbox(t + "_b", ua, ub, LID_T, zt_, xc + h, xb)
            lidbox(t + "_c", ua, uc - h, LID_T, zt_, xc - h, xc + h)
            lidbox(t + "_d", uc + h, ub, LID_T, zt_, xc - h, xc + h)
        for e in (1, -1):                                       # stacking lug at each outer corner, on the hinge side
            lidbox(f"lug_{'ew'[e < 0]}", LUG_U0, LUG_U0 + LUG[1], LID_T, LID_T + LUG[2], *sorted((e * LUG_X0, e * (LUG_X0 + LUG[0]))))
        for k in range(KNUCKLES):                               # visual hinge knuckles centred on the axis
            kx = -(LID_LEN / 2 - KNUCKLE_LEN / 2 - 0.004) + k * (LID_LEN - KNUCKLE_LEN - 0.008) / (KNUCKLES - 1)
            a.cylinder(f"{body}/knuckle_{k}", (kx, 0.0, 0.0), KNUCKLE_R, KNUCKLE_LEN, "X", lid_grey)
        for e in (1, -1):                                       # two round bosses on the top face (visual only)
            a.cylinder(f"{body}/boss_{'ew'[e < 0]}", (e * BOSS_X, -side * BOSS_U, LID_T + BOSS_H / 2), BOSS_R, BOSS_H, "Z", lid_grey)
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{body}/{jname}")
        j.CreateAxisAttr("X")
        j.CreateBody0Rel().SetTargets([Sdf.Path(tub)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(body)])
        j.CreateLocalPos0Attr(Gf.Vec3f(0.0, side * HINGE_Y, HEIGHT))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        lo, hi = (-math.degrees(THETA_OPEN), 0.0) if side > 0 else (0.0, math.degrees(THETA_OPEN))
        j.CreateLowerLimitAttr(float(lo))
        j.CreateUpperLimitAttr(float(hi))
        drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(float(LID_DAMPING) * math.radians(1.0))   # per degree
    UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(tub))
    art = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(tub))
    art.CreateEnabledSelfCollisionsAttr(True)             # the two lids must collide with each other
    art.CreateSolverPositionIterationCountAttr(32)
    art.CreateSolverVelocityIterationCountAttr(8)
    return a.root


def _tote_object(spec: Tote):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class ToteSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_tote)
        kit_name: str = ""

    class KitObject(Object):
        def _generate_articulation_cfg(self):
            actuators = {"hinges": ImplicitActuatorCfg(joint_names_expr=list(SIDES), stiffness=0.0, damping=LID_DAMPING)}
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
            cfg = self._add_initial_pose_to_cfg(cfg)
            cfg.init_state.joint_pos = {"lid_pos": -THETA_OPEN, "lid_neg": THETA_OPEN}
            return cfg

    _SPECS[spec.name] = spec
    return KitObject(name=spec.name, object_type=ObjectType.ARTICULATION,
                     spawner_cfg=ToteSpawnerCfg(kit_name=spec.name),
                     initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos),
                                       rotation_xyzw=tuple(float(v) for v in spec.quat)))


# -- run time ------------------------------------------------------------------------------------------------------------

def _t(x):
    return getattr(x, "torch", x)


def _np(x) -> np.ndarray:
    if hasattr(x, "numpy") and not hasattr(x, "detach"):
        return np.asarray(x.numpy(), dtype=np.float64)
    return _t(x).detach().cpu().numpy().astype(np.float64)


def lid_angles(environment, name: str) -> tuple[dict, dict]:
    """Opening angle theta (rad) and angular speed (rad/s) of each lid."""
    asset = environment.unwrapped.scene[name]
    names = list(asset.joint_names)
    q = _np(asset.data.joint_pos)[0]
    qd = _np(asset.data.joint_vel)[0]
    th = {j: -SIDES[j] * float(q[names.index(j)]) for j in SIDES}        # +y lid: sim angle is -theta; -y lid: +theta
    om = {j: abs(float(qd[names.index(j)])) for j in SIDES}
    return th, om


def _write_pose(environment, name, pos, quat_xyzw) -> None:
    import torch

    base = environment.unwrapped
    asset = base.scene[name]
    origin = _np(base.scene.env_origins[0])
    dev = _t(asset.data.root_link_pose_w).device
    pose = torch.tensor([[*(np.asarray(pos, float) + origin), *quat_xyzw]], dtype=torch.float32, device=dev)
    vel = torch.zeros((1, 6), dtype=torch.float32, device=dev)
    for stem, kw in (("write_root_pose_to_sim", {"root_pose": pose}), ("write_root_velocity_to_sim", {"root_velocity": vel})):
        fn = getattr(asset, f"{stem}_index", None)
        if fn is not None:
            fn(env_ids=[0], **kw)
        else:
            getattr(asset, stem)(*kw.values())


def _set_open(environment, name: str, spec) -> None:
    """Both lids hanging open at the hinge stop, at rest, the tote back on its mark (writing joint positions moves an
    articulation's root, so the root pose is written again after them)."""
    import torch

    asset = environment.unwrapped.scene[name]
    names = list(asset.joint_names)
    ids = [names.index(j) for j in SIDES]
    dev = _t(asset.data.joint_pos).device
    pos = torch.tensor([[-SIDES[j] * THETA_OPEN for j in SIDES]], dtype=torch.float32, device=dev)
    fn = getattr(asset, "write_joint_state_to_sim_index", None)
    if fn is not None:
        fn(position=pos, velocity=torch.zeros_like(pos), joint_ids=ids, env_ids=[0])
    else:
        asset.write_joint_state_to_sim(pos, torch.zeros_like(pos), joint_ids=ids)
    tfn = getattr(asset, "set_joint_position_target_index", None)
    if tfn is not None:
        tfn(target=pos, joint_ids=ids, env_ids=[0])
    else:
        asset.set_joint_position_target(pos, joint_ids=ids)
    _write_pose(environment, name, spec.pos, spec.quat)


SETTLE_STEPS = 10
_SETTLE: dict = {}


def _kit_state(environment, spec) -> ToteState:
    base = environment.unwrapped
    asset = base.scene[spec.name]
    origin = _np(base.scene.env_origins[0])
    pose = _np(asset.data.root_link_pose_w)[0]
    pos, quat = pose[:3] - origin, pose[3:7]
    vel = _np(asset.data.root_vel_w)[0] if hasattr(asset.data, "root_vel_w") else np.zeros(6)
    th, om = lid_angles(environment, spec.name)
    corners = np.array([[x, y, z] for x in (-LENGTH / 2, LENGTH / 2) for y in (-WIDTH / 2, WIDTH / 2) for z in (0.0, HEIGHT)])
    pts = corners @ _rotation(quat).T + pos
    return ToteState(pos=pos, quat_xyzw=quat, lin_vel=vel[:3], ang_vel=vel[3:6], low=pts.min(0), high=pts.max(0),
                     joints=th, joint_vel=om)


def finger_points(environment) -> list:
    """The grasp point of each gripper (world, relative to the env origin), from the arms' joint angles."""
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS

    base = environment.unwrapped
    pts = []
    for arm in ("left", "right"):
        robot = base.scene[f"{arm}_robot"]
        names = list(robot.joint_names)
        q = _np(robot.data.joint_pos)[0]
        qa = [float(q[names.index(j)]) for j in ARM_JOINTS]
        pts.append(ik.tcp_pose(np.array(qa), station.BASES[arm])[0])
    return pts


# -- hooking yam_arena ------------------------------------------------------------------------------------------------

_STEP: list = []
_RESET: list = []
_ENV: dict = {"env": None}


def on_step(fn):
    _STEP.append(fn)
    return fn


def on_reset(fn):
    _RESET.append(fn)
    return fn


def _patch_scene(scene) -> None:
    if getattr(scene, "_totekit_patched", False):
        return
    stock_object, stock_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Tote):
            return _tote_object(spec)
        return stock_object(spec)

    def object_states(environment, names):
        mine = {n for n in names if isinstance(_SPECS.get(n), Tote)}
        plain = [n for n in names if n not in mine]
        out = stock_states(environment, plain) if plain else {}
        for n in mine:
            out[n] = _kit_state(environment, _SPECS[n])
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._totekit_patched = True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_totekit_patched", False):
        return
    stock_step, stock_reset = cls._task_step, cls.reset

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        for name, left in list(_SETTLE.items()):                       # just reset: hold it open where it stands
            if left > 0:
                _SETTLE[name] = left - 1
                _set_open(self._env, name, _SPECS[name])
        for fn in _STEP:
            fn(self._env)
        return stock_step(self, *a, **k)

    def reset(self, *a, **k):
        obs = stock_reset(self, *a, **k)
        _ENV["env"] = self._env
        for name, spec in _SPECS.items():
            _set_open(self._env, name, spec)
            _SETTLE[name] = SETTLE_STEPS
        try:                                         # the observation above was drawn before the lids were opened: draw it again
            base = self._env.unwrapped
            for _ in range(3):
                for name, spec in _SPECS.items():
                    _set_open(self._env, name, spec)
                base.sim.step(render=True)
            base.scene.update(dt=base.physics_dt)
            obs = self._observation()
        except Exception as exc:                     # keep the stock observation
            print(f"totekit: could not refresh the reset observation: {exc!r}", file=sys.stderr)
        for fn in _RESET:
            fn(self._env)
        return obs

    cls._task_step, cls.reset = _task_step, reset
    cls._totekit_patched = True


class _Finder(importlib.abc.MetaPathFinder):
    targets = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment}

    def find_spec(self, fullname, path, target=None):
        if fullname not in self.targets:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        loader, patch = spec.loader, self.targets[fullname]
        stock_exec = loader.exec_module

        def exec_module(module):
            stock_exec(module)
            patch(module)

        loader.exec_module = exec_module
        return spec


def install() -> None:
    """Hook yam_arena (now for modules already imported, otherwise when they are). Idempotent."""
    for name, patch in _Finder.targets.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
