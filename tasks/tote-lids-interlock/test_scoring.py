"""Offline checks of the scoring rule and the tote's geometry (no simulator; needs yam_arena importable):
python -m pytest test_scoring.py"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import task as T  # noqa: E402
import totekit as K  # noqa: E402

D = math.radians


@pytest.fixture(autouse=True)
def fresh():
    T._clear()
    yield
    T._clear()


def g(a, b, **kw):
    kw = {"moved": False, "tipped": False, "held": False, **kw}
    return T.grade(D(a), D(b), **kw)


# -- grading: every outcome ------------------------------------------------------------------------------------------
def test_open_lids_score_zero():
    assert g(180, 180) == 0.0


def test_one_lid_seated_other_open_scores_a_quarter():
    assert g(0, 180) == 0.25 and g(180, 0) == 0.25
    assert g(0, 10.1) == 0.25                       # the other lid open 10 degrees or more


def test_one_lid_down_but_not_seated_other_open_scores_zero():
    assert g(5, 180) == 0.0 and g(180, 9.9) == 0.0


def test_one_seated_one_propped_scores_half():
    assert g(0, 6.7) == 0.5 and g(6.7, 0) == 0.5 and g(0, 9.9) == 0.5


def test_both_down_not_seated_scores_a_quarter():
    assert g(5, 5) == 0.25 and g(9.9, 9.9) == 0.25


def test_both_seated_without_the_hold_scores_a_quarter():
    assert g(0, 0) == 0.25


def test_full_score_needs_the_hold():
    assert g(0, 0, held=True) == 1.0


def test_grippers_not_clear_at_the_end_scores_zero():
    for a, b in ((0, 0), (0, 6.7), (0, 180), (5, 5)):
        assert g(a, b, clear=False) == 0.0


def test_holding_the_lids_down_with_a_gripper_at_timeout_scores_zero():
    """Both lids pressed flat, a gripper still on the tote when the run ends: the hold never completes, grippers not clear."""
    T._STATE.update(clear=False, held_steps=0, done=False)
    objs = {T.NAME: type("S", (), {"joints": {"lid_pos": 0.0, "lid_neg": 0.0}})()}
    assert T.score(objs) == 0.0


def test_propped_case_through_score():
    T._STATE.update(clear=True, held_steps=0, done=False)
    objs = {T.NAME: type("S", (), {"joints": {"lid_pos": 0.0, "lid_neg": D(6.7)}})()}
    assert T.score(objs) == 0.5


def test_both_seated_grippers_clear_held_scores_one_through_score():
    T._STATE.update(clear=True, held_steps=T.HOLD_STEPS, done=True)
    objs = {T.NAME: type("S", (), {"joints": {"lid_pos": 0.0, "lid_neg": 0.0}})()}
    assert T.score(objs) == 1.0 and T.success(objs)


def test_moved_or_tipped_scores_zero_whatever_the_lids_do():
    assert g(0, 0, held=True, moved=True) == 0.0
    assert g(0, 0, held=True, tipped=True) == 0.0
    assert g(5, 5, moved=True) == 0.0


def test_every_outcome_is_a_listed_score():
    for a in (0, 0.5, 2, 5, 9.9, 10, 40, 180):
        for b in (0, 0.5, 2, 5, 9.9, 10, 40, 180):
            for clear in (True, False):
                assert g(a, b, clear=clear) in (0.0, 0.25, 0.5)


def test_seated_means_the_gauge_edge_is_under_3_mm():
    for side in (+1, -1):
        th = math.asin(0.003 / K.gauge_u(side))
        assert T.seated(side, th * 0.99) and not T.seated(side, th * 1.01)
        assert K.edge_height(side, 0.0) == 0.0
    assert 0.5 < math.degrees(math.asin(0.003 / K.gauge_u(+1))) < 1.0
    assert K.GAUGE_X[+1] < 0 < K.GAUGE_X[-1]            # the +y lid is the wider one in the -x half


# -- clearance of the grippers ------------------------------------------------------------------------------------------
def test_box_distance():
    o = T.start()
    top = o[2] + K.HEIGHT + K.LIP_LEVEL + K.LID_T
    assert T.box_distance(np.array([o[0], o[1], top + 0.05]), o) == pytest.approx(0.05)
    assert T.box_distance(np.array([o[0], o[1] + K.WIDTH / 2 + 0.04, o[2] + 0.1]), o) == pytest.approx(0.04)
    assert T.box_distance(np.array([o[0], o[1], o[2] + 0.1]), o) == 0.0


def test_tilt():
    assert T.tilt((0, 0, 0, 1)) == pytest.approx(0.0)
    s = math.sin(D(10) / 2)
    assert T.tilt((s, 0, 0, math.cos(D(10) / 2))) == pytest.approx(D(10))


# -- the tote ------------------------------------------------------------------------------------------------------------
def test_outside_dimensions_are_the_catalogue_ones():
    assert (K.LENGTH, K.WIDTH, K.HEIGHT) == pytest.approx((21.5 * 0.0254, 15 * 0.0254, 9 * 0.0254), abs=1e-3)


def test_each_lid_is_wider_in_one_half_and_the_zones_tile_the_lid():
    for side in (+1, -1):
        zones = sorted(K.lid_zones(side), key=lambda z: z["x0"])
        assert zones[0]["x0"] == pytest.approx(-K.LID_LEN / 2) and zones[-1]["x1"] == pytest.approx(K.LID_LEN / 2)
        for z0, z1 in zip(zones, zones[1:]):
            assert z0["x1"] == pytest.approx(z1["x0"])
    left, right = ({id_: z for id_, z in enumerate(sorted(K.lid_zones(s), key=lambda z: z["x0"]))} for s in (+1, -1))
    # the +y lid carries the flange in the -x half, the -y lid in the +x half
    assert left[0]["flange"] is not None and right[0]["flange"] is None
    assert left[len(left) - 1]["flange"] is None and right[len(right) - 1]["flange"] is not None


def test_the_flange_overlaps_the_other_lids_tongue_with_a_2mm_gap_in_each_half():
    for zone_x in (-0.1, +0.1):
        yc = K.centre_y(zone_x)
        wide = +1 if zone_x < 0 else -1
        z_w = next(z for z in K.lid_zones(wide) if z["x0"] <= zone_x <= z["x1"])
        z_n = next(z for z in K.lid_zones(-wide) if z["x0"] <= zone_x <= z["x1"])
        edge_w = wide * (K.HINGE_Y - z_w["u_end"])             # the wider lid's plate edge (world y)
        tip_n = -wide * (K.HINGE_Y - z_n["u_end"])             # the narrower lid's tongue tip
        flange_tip = wide * (K.HINGE_Y - z_w["flange"])
        assert edge_w == pytest.approx(yc + wide * K.CENTRE_GAP)
        assert flange_tip == pytest.approx(yc - wide * K.OVERLAP)        # reaches past the seam over the other lid
        assert abs(edge_w - tip_n) == pytest.approx(0.002, abs=1e-6)       # the tongue stops 2 mm short of the plate edge/riser
        assert (flange_tip - tip_n) * wide < 0                           # the tongue lies under the flange


def test_the_tooth_passes_over_the_other_lids_tongue_with_an_air_gap():
    assert K.LIP_LEVEL - K.LID_T >= 0.010           # air gap between a tongue's top and a finger's underside
    assert K.FINGER_REACH > K.HINGE_Y               # fingers reach past the centre line
    assert K.TONGUE_REACH > K.HINGE_Y               # so do tongues, stopping short of the other lid's riser
    assert K.TONGUE_REACH < K.HINGE_Y + K.CENTRE_GAP


def test_stepped_seam_jams_a_second_lid_and_lets_two_lids_down_together():
    """2D cross-section of the lids in both halves of the stepped seam (lip2d.py; see README)."""
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    import lip2d

    lip2d.D, lip2d.C, lip2d.OV, lip2d.PS = K.SEAM_OFFSET, K.CENTRE_GAP, K.OVERLAP, K.TONGUE_STOP
    assert not lip2d.collide(0, 0, K.LIP_LEVEL)
    jam = lip2d.jam_angle(K.LIP_LEVEL)
    assert 5.5 < jam < 8.0                                  # the second lid rests propped about 6-7 degrees open
    for lag in (0.0, 1.0, 2.0):                             # together: either lid may lead by up to 2 degrees
        assert all(not lip2d.collide(ta, max(ta - lag, 0), K.LIP_LEVEL) for ta in np.arange(40, -0.01, -0.2))
        assert all(not lip2d.collide(max(ta - lag, 0), ta, K.LIP_LEVEL) for ta in np.arange(40, -0.01, -0.2))


def test_closed_lid_edge_sits_at_the_rim():
    for side in (+1, -1):
        p = K.lid_point(side, 0.0, K.PLATE_LEN)
        assert p[1] == pytest.approx(side * K.CENTRE_GAP) and p[2] == pytest.approx(K.HEIGHT)


def test_open_flat_lid_points_outward_and_the_face_normal_points_down():
    p = K.lid_point(+1, math.pi, 0.2, offset=0.01)
    assert p[1] == pytest.approx(K.HINGE_Y + 0.2) and p[2] == pytest.approx(K.HEIGHT - 0.01)


def test_start_layout_is_fixed_and_reachable():
    assert T.OBJECTS[0].pos == (T.TOTE_XY[0], 0.0, T.TOP)
    # nearest lid end is clear of the arms' wrists; far end inside the table
    assert T.TOTE_XY[0] - K.LENGTH / 2 > 0.24 and T.TOTE_XY[0] + K.LENGTH / 2 < 0.9


# -- product-photo details (all estimated, see README) -------------------------------------------------------------------
def _boxes(parts, collide=True):
    return [p for p in parts if p[4] == collide]


def test_tub_tapers_in_and_the_rim_band_is_the_catalogue_outline():
    assert K.A_BASE > K.A_TOP > 0
    assert K.inset(0.0) == pytest.approx(K.A_BASE) and K.inset(K.ZT) == pytest.approx(K.A_TOP)
    assert 2 * K.A_BASE < 0.06                                       # base footprint 2.5 cm in on each side
    assert 3.0 < math.degrees(K.PHI) < 8.0
    parts = {p[0]: p for p in K.tub_parts()}
    rim = parts["rim_pos"]
    assert rim[1][2] + rim[2][2] / 2 == pytest.approx(K.HEIGHT)      # the rim top is the lids' rest plane, as before
    assert rim[1][1] + rim[2][1] / 2 == pytest.approx(K.WIDTH / 2)   # band outline = the catalogue outside width
    assert 0.03 <= K.RIM_H <= 0.04


def test_tub_collision_parts_stay_inside_the_outline_and_start_at_the_table():
    lo = hi = 0.0
    for name, c, size, q, collide in K.tub_parts():
        if not collide:
            continue
        assert abs(c[0]) + size[0] / 2 <= K.LENGTH / 2 + 1e-6, name
        assert abs(c[1]) + size[1] / 2 <= K.WIDTH / 2 + 1e-6 or name.startswith("wall_end"), name
        lo, hi = min(lo, c[2] - size[2] / 2 * math.cos(K.PHI)), max(hi, c[2] + size[2] / 2)
    assert lo > -1e-3 and hi <= K.HEIGHT + 1e-6


def test_slots_are_real_gaps_and_ribs_are_visual_only():
    parts = K.tub_parts()
    assert all(not p[4] for p in parts if p[0].startswith("rib_"))
    assert sum(p[0].startswith("rib_long") for p in parts) == 2 * 6          # 5 panels per long wall -> 6 ribs
    assert sum(p[0].startswith("rib_end") for p in parts) == 2 * 1           # 2 panels per end wall -> 1 inner rib (corners shared)
    for tag in ("wall_long_p", "wall_end_p"):                                # the slot band is split into segments around the slots
        assert sum(p[0].startswith(tag + "_mid") for p in parts) >= 3 or tag == "wall_end_p"
    assert K.ZT - K.SLOT_GAP - K.SLOT_H > 0.1


def test_lid_additions_do_not_change_the_mechanism_numbers():
    assert K.HINGE_Y == pytest.approx(K.WIDTH / 2 + K.HINGE_OUT)
    assert K.LUG_U0 + K.LUG[1] < 0.06 and K.LUG_X0 + K.LUG[0] <= K.LID_LEN / 2          # lugs sit well clear of the pinch (u 0.10, x -0.259)
    assert K.LID_T + K.LUG[2] < K.LID_T + K.LIP_LEVEL                                     # no taller than the flanges
    assert K.HOLE < min(K.BLOCK[0], K.BLOCK[1]) / 2
