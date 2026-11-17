"""On the station table: an Akro-Mils 39085 attached-lid tote (a flip-top container, 21.5 x 15 x 9 in), standing with its long
side along x. Its two lids are hinged on its long sides (+y and -y) and lie open, flat and outward, on their hinge stops.
The lids meet along a stepped seam (each is wider in one half) where they lap over each other.

Both lids have to end up shut, flat and seated on the rim, with the grippers withdrawn. Each arm can reach its own side's lid.

Run it:    inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:      inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import totekit as K  # noqa: E402
from totekit import Tote, finger_points, on_reset, on_step  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0                 # the task holds its own 2 s
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Close the lids of the tote."
NAME = "tote"
TOTE_XY = (0.52, 0.0)                # middle of the tote's footprint, m from the midpoint between the arm bases

# -- scoring ---------------------------------------------------------------------------------------------------------
MOVED = 0.03                         # m: the tote slid this far (anywhere) from its mark
TIPPED = math.radians(10.0)          # the tote tilted this far at any time
DOWN = math.radians(10.0)            # a lid is "down" when it is open by less than this
SEATED_MM = 3.0                      # a lid is seated when its free edge at the gauge point is within this of the rim plane
HOLD_STEPS = 40                      # flush, still and clear this many control steps (2 s at 20 Hz)
CLEAR = 0.030                        # m: each gripper's tool point this far from the closed tote
STILL = 0.05                         # rad/s: a lid hinge slower than this is at rest

OBJECTS = [Tote(NAME, pos=(TOTE_XY[0], TOTE_XY[1], TOP), size=(K.LENGTH, K.WIDTH, K.HEIGHT), mass=K.TOTE_MASS)]


def start() -> np.ndarray:
    return np.array([TOTE_XY[0], TOTE_XY[1], TOP])


# -- grading (pure) ---------------------------------------------------------------------------------------------------
def seated(side: int, theta: float) -> bool:
    """The ONE measuring point: the height of the lid's free (seam-side) plate edge above the rim plane, at the middle of its
    straight seam section (K.GAUGE_X), is under 3 mm, with the lid on the tote (not open flat outward). Real world: straightedge across the rim, ruler under the lid's edge."""
    return theta < math.pi / 2 and K.edge_height(side, theta) * 1000.0 < SEATED_MM     # (a lid open flat outward is also at rim height)


def grade(th_pos: float, th_neg: float, *, moved: bool, tipped: bool, held: bool, clear: bool = True) -> float:
    """Score from the lids' opening angles (rad) at the end, and what happened during the trial.
    0 if the tote slid or tilted; 1.0 if held (both seated, still, grippers clear, 2 s). Otherwise, only if both grippers are
    clear at the end (else 0): 0.5 if one lid is seated and the other is down but not seated (open under 10 degrees);
    0.25 if exactly one lid is seated (the other open 10 degrees or more), or both are down (open under 10 degrees) and the
    hold was not completed; 0 in every other case."""
    if moved or tipped:
        return 0.0
    if held:
        return 1.0
    if not clear:
        return 0.0
    sp, sn = seated(+1, th_pos), seated(-1, th_neg)
    dp, dn = th_pos < DOWN, th_neg < DOWN
    if (sp and dn) or (sn and dp):
        return 0.25 if (sp and sn) else 0.5
    if sp or sn:
        return 0.25
    return 0.25 if (dp and dn) else 0.0


def tilt(quat) -> float:
    return math.acos(max(-1.0, min(1.0, float(K._rotation(quat)[2, 2]))))


def box_distance(point, origin) -> float:
    """Distance from ``point`` to the closed tote's outline (a box with the lids shut)."""
    lo = np.array([origin[0] - K.LENGTH / 2, origin[1] - K.WIDTH / 2, origin[2]])
    hi = np.array([origin[0] + K.LENGTH / 2, origin[1] + K.WIDTH / 2, origin[2] + K.HEIGHT + K.LIP_LEVEL + K.LID_T])
    d = np.maximum(np.maximum(lo - point, point - hi), 0.0)
    return float(np.linalg.norm(d))


# -- what happened during the trial ---------------------------------------------------------------------------------
_STATE: dict = {}


def _clear() -> None:
    _STATE.update(clear=True, moved=False, tipped=False, held_steps=0, done=False, steps=0, lag=0.0)


_clear()


@on_reset
def _reset(environment) -> None:
    _clear()


@on_step
def _watch(environment) -> None:
    from yam_arena.scene import object_states   # importable only once the simulator has started

    s = _STATE
    s["steps"] += 1
    st = object_states(environment, [NAME])[NAME]
    pos = np.asarray(st.pos)
    if s["steps"] > 15:                                  # after the lids have settled
        if float(np.hypot(*(pos[:2] - start()[:2]))) > MOVED:
            s["moved"] = True
        if tilt(st.quat_xyzw) > TIPPED:
            s["tipped"] = True
    th, om = st.joints, st.joint_vel
    if max(th.values()) < math.radians(30.0):           # measured, for the README: how far apart the lids were while meeting
        s["lag"] = max(s["lag"], abs(th["lid_pos"] - th["lid_neg"]))
    flat = seated(+1, th["lid_pos"]) and seated(-1, th["lid_neg"]) and max(om.values()) < STILL
    clear = all(box_distance(p, start()) >= CLEAR for p in finger_points(environment))
    s["clear"] = clear
    ok = flat and clear and not s["moved"] and not s["tipped"]
    s["held_steps"] = s["held_steps"] + 1 if ok else 0
    s["done"] = s["done"] or s["held_steps"] >= HOLD_STEPS
    if os.environ.get("TOTE_TRACE") and s["steps"] % 20 == 0:
        print(f"TRACE step {s['steps']} lids {math.degrees(th['lid_pos']):.1f} {math.degrees(th['lid_neg']):.1f} deg, "
              f"tote {np.round(100 * (pos - start()), 2).tolist()} cm, tilt {math.degrees(tilt(st.quat_xyzw)):.2f}, clear {clear}, "
              f"maxdiff {math.degrees(s['lag']):.1f} held {s['held_steps']} moved {s['moved']} tipped {s['tipped']}", flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------
def score(objects) -> float:
    """0 for the whole trial if the tote ever slid more than 3 cm from its mark or tilted 10 degrees or more. 1.0 once both
    lids are seated (each lid's free edge at the middle of its straight seam section within 3 mm of the rim plane), still,
    with both grippers 3 cm or more from the tote, held 2 s. Otherwise at the end, and only if both grippers are 3 cm or more
    from the tote (else 0): 0.5 if one lid is seated and the other is propped on its lap (open under 10 degrees); 0.25 if
    exactly one lid is seated, or both are down (open under 10 degrees) without the hold; 0 in every other case."""
    s = _STATE
    th = objects[NAME].joints
    return grade(th["lid_pos"], th["lid_neg"], moved=s["moved"], tipped=s["tipped"], held=s["done"], clear=s["clear"])


def success(objects) -> bool:
    return score(objects) >= 1.0


def report(objects) -> dict:
    st = objects[NAME]
    j = st.joints
    return {"score": score(objects), "moved": _STATE["moved"], "tipped": _STATE["tipped"], "held_steps": _STATE["held_steps"],
            "tote_cm": [round(100 * float(v), 2) for v in st.pos], "tilt_deg": round(math.degrees(tilt(st.quat_xyzw)), 2),
            "lid_pos_deg": round(math.degrees(j["lid_pos"]), 1), "lid_neg_deg": round(math.degrees(j["lid_neg"]), 1)}


def demo():
    """The scripted two-arm plan (``demo.py``), played by ``--policy yam_scripted``."""
    import demo as D

    D.install()
    import types

    return D.plans(types.SimpleNamespace(start=start))
