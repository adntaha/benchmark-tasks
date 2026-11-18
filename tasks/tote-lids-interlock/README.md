# Close an attached-lid tote: lower both lids together so the lips along their seam seat flush

**Instruction:** "Close the lids of the tote."

An Akro-Mils 39085 attached-lid container (a flip-top tote, 21.5 x 15 x 9 in outside) stands on the table with its long side
along x. Its two lids are hinged on the long sides. **The product's open lids do not stay flat: they swing about 270 degrees
from closed and hang against the outside of the long wall (manufacturer's video, https://youtu.be/B8cnZ48xBc0, and a maker's product image of four totes, two with their lids open and hanging).** The task
therefore starts with both lids lying flat and outward (180 degrees, top faces down), each resting on a fixed support beside the
tote; the supports are part of the setup, not of the product. The robot (both YAM arms, parallel grippers) has to lift each lid off
its support, shut both lids, level and seated on the rim, withdraw both grippers, and leave the tote on its mark.

The lids meet along an S-shaped seam traced from product photos (see "The seam": four straight runs and three diagonals; each
lid has one full tooth and a half tooth). In the photos their tops are flush across the seam. The model has no snap or latch; a shut lid is held by its weight.

**The mechanism (the product's design as described by the team).** Along the seam each lid's edge alternates between an UPPER lip and
a LOWER lip, and the two lids' lips interleave: where one lid has the upper lip the other has the lower lip. An upper lip has to slide
on top of the lower lip. If one lid is lowered first, the second lid's lower lip lands on top of the first lid's upper lip, one step
(one plate thickness, 4 mm, estimated) above where it must go, and the lids cannot close; so the lids have to come down together. **This
layering is not visible from above in the photos.** The step, the lap lengths and which lid overlies in each section are estimates. In the
model (a hidden lap, below) either closing order leaves the second lid resting on the first lid's upper lip. Two videos are cited, each
only for what the team sees in it: in the seller's product video (https://www.amazon.ca/vdp/09c3c1c46a40487390129b9b26503d1c, about
0:19-0:25) both lids are shut at the same time; the manufacturer's video (https://youtu.be/B8cnZ48xBc0) and a maker's product image of four totes (two of them open) show the lids hanging.

## Setup (one fixed layout; all positions in cm from the midpoint between the two arm bases, x forward, y left)

The screenshot (`setup.png`, 1280x720) is a render-only view from the far side of the table, higher and wider than the policy's top camera, so
that the whole tote, both lids and both supports show; the part of each support's top under its lid is hidden. It does not change the policy's
cameras, observations or score.

* **Arm bases:** x 0, y +30.5 cm (left arm) and y -30.5 cm (right arm), on the table surface (the origin is the midpoint between them).
* **Tote:** standing loose on its base, long side along x, at its mark (the tote is not fixed, and sliding it is scored). The mark is the outline of the tote's base, taped on the table (49.6 x 33.1 cm, which is 2.5 cm inside the rim outline on every side), centred at x 52, y 0.
* **Supports (two):** plywood boxes (or stacked lumber), each 38.4 cm long (x 35.1 to 73.5) and 13.0 cm wide, one at
  y +31.5 to +44.5 (under the left lid), one at y -44.5 to -31.5 (under the right lid), with a flat top 22.46 cm above the
  table (the design height), taped to the table at their marks. A lid lies on a support with its top face down, from 12 cm to 25 cm out from its
  hinge axis.
* **Why these shapes:** the support starts 12 cm from the hinge axis, so it is 2.0 cm or more clear of the two bosses (3 mm
  proud, 6.8 to 9.2 cm out) and clear of the embossed logo plates (about 6 to 11.5 cm out, read from the retailer product image of the grey 39085); a lid resting on a boss
  would sit 3 mm high. The support ends at x 73.5, 2.4 cm short of the far end's seal block (and clear of the lug), and it reaches 2.5 cm or
  more past the outer edge of the lid, so orange shows around the lid in the screenshot. The lid's weight acts about 10 cm from the hinge
  axis, which is NOT over the support (the support starts 12 cm from the hinge axis): the weight
  lies between the hinge axis and the support's near edge, so the hinge and that edge carry the lid and it lies level.
* **Overhang:** each lid's near end (x 25.1) sticks out 10 cm past its support (the support's near end is at x 35.1), so the lower
  finger pad can reach under the plate at the pinch (10 cm out from the hinge). For that reason the support does not extend
  past the lid at the near end.
* **Height check (one check):** shim the support until a digital inclinometer (or phone level app), zeroed on the table beside the
  tote, reads 0.0 +- 0.3 degrees on the lid's upward face (its underside, since the lid lies top face down), laid across the lid's width (along y) on a flat part of the underside between x 40 and 70 cm, with the lid resting on the support. The support top is designed at 22.46 cm above the table. The overhang is checked with a ruler (10 cm, within 1 cm).
* **Both lids** lie flat and outward, on the supports, at the start of every trial and after every reset. They do not stay flat
  by themselves, so a lid let go anywhere else swings down and hangs.

### Why trials do not start from the hanging pose

Offline inverse kinematics (`python demo.py hang`; `yam_arena.ik`, the existing arm layout) was run for a pinch on a lid hanging at
the hinge stop, 2 cm above its free edge at the near end, approached along +x like the demo's pinch. Result: the left lid (its free edge
at that end, a tongue stretch, hangs 6.7 cm above the table) is not reachable (position error 65.9 mm, rotation error 68.3 degrees); the
right lid (free edge 0.5 cm above the table, its wider stretch; pinch point 2.5 cm above the table) is not reachable either
(257 mm, 35.0 degrees). So a pinch on a hanging lid is out of reach for this arm layout, and the pads could not get under a lid
edge that hangs a few millimetres above the table in any case. This is a position and orientation check only, not a collision check.

## The seam: what the photos show, and what is estimated

Sources: the retailer product image of the grey Akro-Mils 39085 (lids closed, seen obliquely from one end), a close-up product image of
a grey tote's closed lids (possibly another model), the manufacturer's page (https://akro-mils.com/product/attached-lid-container-39085/)
and the manufacturer's video (https://youtu.be/B8cnZ48xBc0).

**What the photos show.** The lid tops look flush across the seam: a thin dark line, no raised rim above the lid top. The seam
is not straight: seen from above it has three direction changes, and between them two long straight stretches that lie on opposite sides
of the centre line, joined by oblique diagonals (a short step at each seal block is also visible and is not modelled). Two
round bosses sit on each lid top, hinge knuckles run along both long sides, and raised corner blocks and seal blocks with a hole
sit at the lid ends.

**How it was read.** The four corners of the lid outline in the retailer image of the grey 39085 were used to rectify the photo to a top view (a
homography to a 53.8 x 38.9 cm rectangle), and the seam's breakpoints were read in that view; the bosses and logos were used to
check the scale (the four bosses came out at x +-14 cm and y +-11 cm). Raw readings from that image, x along the lid
from the left end of the photo and y across (cm; the table's coordinates follow by subtracting 1.7 cm in x and adding 1.85 cm in y, which centres the bosses): left end
run at y +0.3 from x -23.6 to -19.5; diagonal to (-14.0, -4.8); long run at y -4.9 to x -1.2; diagonal to (5.1, +0.7); long run at y +1.1 to
x 17.6; diagonal to (23.5, -4.3); end run at y -4.3. The two levels came out about 5 to 6 cm apart in the grey 39085 image and about 6 cm in the close-up image,
from a rectification whose own error is about +-1.5 cm. The traced polyline below is those readings, made point-symmetric about the tote's
centre.

**The modelled seam** (x, y in cm from the tote's centre, y positive to the left; all of it an ESTIMATE, +-1.5 cm; the left end of
the photo is -x, the end nearest the arms):

| Point | x | y |
|---|---|---|
| 1 (left end) | -26.9 | +3.0 |
| 2 | -21.5 | +3.0 |
| 3 | -15.8 | -3.0 |
| 4 | -3.0 | -3.0 |
| 5 | +3.0 | +3.0 |
| 6 | +15.8 | +3.0 |
| 7 | +21.5 | -3.0 |
| 8 (right end) | +26.9 | -3.0 |

So: an end run R0 (5.4 cm, y +3), a diagonal (5.7 cm in x), a long run R1 (12.8 cm, y -3), the short middle diagonal (6.0 cm in x, about a
tenth of the lid length), a long run R2 (12.8 cm, y +3), a diagonal, and an end run R3 (5.4 cm, y -3). The left lid (hinged at +y) reaches
across the centre line in R1 (a full tooth) and R3 (half a tooth at the end); the right lid in R2 (full) and R0 (half). **Each lid has one
full tooth and a half tooth.** The seam is point-symmetric: a half turn about the tote's centre maps it onto itself.

**Hidden lap (ESTIMATED; not visible in any photo).** Nothing in the photos shows how the lids lap under their flush tops, or which lid lies
on top where. The model uses, in each straight run, an overhang (a thin upper layer, below) on the lid that reaches across the
centre line, lying over a thin tongue of the other lid:

* top faces flush (both at the lid's 4 mm plate top); 0.2 cm gap between the top faces along the seam;
* the overlying lid has a full 4 mm plate to 1.2 cm short of its seam-line edge, then an overhang 2.2 mm thick (from 1.8 to 4.0 mm above the
  plate's lower face) out to the seam line;
* the other lid has a full plate to the 0.2 cm gap, then a tongue 1.2 mm thick (0 to 1.2 mm) reaching 1.0 cm past the seam line, under the
  overhang, with 0.6 mm clearance between the tongue's top and the overhang's underside;
* in the diagonals both lids are plain 4 mm plate with straight edges (one convex prism per diagonal and lid, 4 mm thick, flush with the plate top), the edges parallel and 0.2 cm apart measured along y, which is about 1.4 mm across the edge at the diagonals' slope of about 46 to 47.5 degrees; each lid's edge starts on the traced polyline at one end of a diagonal and ends one gap beside it at the other.

In these terms the overhang is the UPPER lip and the tongue the LOWER lip. The overhang alternates along the seam (R1 and R3: +y lid; R0 and R2: -y lid), so each lid has the upper lip in some runs and the lower lip in others, as in the team's description of the product. Which lid overlies in each section of the real product, the step (one plate thickness, 4 mm, estimated) and the lap lengths are **not known**; they are estimates, and the layering is not visible from above in the photos.
Collision boxes only, except the diagonals' plate prisms (convex hulls). The seam offset is limited by the hang test: the overlying lid's top edge reaches 22.45 cm from its hinge axis, which
must stay 3 mm under the 22.86 cm tote height so that a hanging lid clears the table.

### Does closing one lid first jam in the model?

Yes in the model, but at a small angle, and the angle is not the point: the step is. A lid shut first leaves the second one resting on
it, open 1.4 degrees in the simulator (`TOTE_MODE=seq`, left lid shut first; `lip2d.py`, the offline 2D cross-section, gives 1.3
degrees in either order). This is the model's version of the interleaved lips: the second lid's lip lands on the first lid's upper lip,
about one step high (the step is one plate thickness, 4 mm, estimated; in the model the propped state is 1.4 degrees, which is 5.5 mm
at the measuring point: 22.45 cm * sin 1.4 degrees; not measured on the product).
Because the overlying lid alternates along the seam, either closing order blocks. Two lids that come down at the same angle never touch
(lip2d.py checks every angle from 40 to 0 degrees); with one lid leading by 0.5 degrees or more in the last few degrees the lip
catches. That is the consequence of the estimated lap, not something the photos show, and nothing was added to the geometry to force it.

**Why 0.5 degrees.** The limit separates a seated lid (0.0 degrees in the model) from the lid resting on the other lid's lip (1.4 degrees in the
model, 5.5 mm at the measuring point). A digital inclinometer's accuracy is about +-0.2 degrees (an estimate), so 0.5 degrees is
clear of both states in the model. See Scoring for the check.

## Why it matters

O*NET 53-7065.00, Stockers and Order Fillers (2,804,300 US employees in 2025, BLS data as shown on O*NET), lists as core
tasks "Pack and unpack items to be stocked on shelves in stockrooms, warehouses, or storage yards." and "Receive, unload,
open, unpack, or issue sales floor merchandise." Closing a tote's lids is part of that
handling, not a separately listed task.

## Files

| File | What |
|---|---|
| `task.py` | the task file: scene (tote and the two fixed supports), scoring, `demo()` |
| `totekit.py` | the tote as one articulation (tub root, two revolute lid hinges with range 0 to 265 degrees, cuboid colliders for the tapered walls, rim band, foot, every straight-run plate, overhang, tongue, seal block, lug and boss, and a convex prism for each diagonal plate piece; visual-only ribs and knuckles), the supports' geometry and the hooks yam_arena needs to spawn it |
| `demo.py` | the scripted two-arm plan (open loop), the control-run modes and the hanging-pose IK check (`python demo.py hang`) |
| `lip2d.py` | offline 2D cross-section check of the lap (prop angle, lead tolerance) |
| `test_scoring.py` | offline tests of scoring, geometry, hinge range, supports and the seam |
| `task.json` | the submission fields |

## Run and score a trial

On a machine with the `yam_arena` rig (Isaac Sim, Isaac Lab-Arena 0.3.1), from this folder:

    inspect-robots run --embodiment yam_arena -E task=task.py -E scene=station -E object_jitter=0 -E control_hz=20 \
      -E cam_width=320 -E cam_height=176 --instruction "Close the lids of the tote." \
      --scorer task_score --max-steps 1800 --store-frames --log-dir runs/trial

Scripted demo (open loop, from the fixed layout; no simulator state is read, nothing attached, no helper forces):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E scene=station -E object_jitter=0 \
      -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "Close the lids of the tote." \
      --scorer task_score --max-steps 1800 --store-frames --log-dir runs/demo
    inspect-robots video runs/demo/adhoc*.json

`task_score` reports 1.0 on success, otherwise the partial credit. Environment variables for control runs:
`TOTE_MODE=both|nopress|stay|seq|left|right`, `TOTE_LAG=<seconds>` (holds the right arm back), `TOTE_TRACE=1` (prints lid
angles and tote drift every 20 steps). Unit tests: `python -m pytest test_scoring.py` with `yam_arena` importable.

**Dream-1 run: 0.0** (1,800 steps; it never closed a lid), run once on this layout with the instruction above and the 1,800-step limit.

**Runtime per trial:** the demo reaches full score at step 366 (18.3 s of simulated time at 20 Hz; the run ends there), well inside the
1,800-step limit; the one-lid-first control runs the full plan. **Seeds:** none; `-E object_jitter=0` fixes the layout (the
supports are fixed objects and are never jittered).

## Scoring (also in `task.json`)

0 for the whole trial if the tote ever slides more than 3 cm from its mark or tilts more than 10 degrees from its starting reading (a two-axis inclinometer or phone level app taped to the tote's end wall, zeroed on the table before the trial, read from video). The supports are fixed to the table; only the tote is checked. The tote's mark is the outline of its base on the table.

**Seated (one definition, in the code and in the text):** a lid is seated when the tilt of its top face across its width (hinge side
to seam side) is below 0.5 degrees from horizontal. Real-world check: place a digital inclinometer (or a phone level app) on a flat part
of the lid top at the lid's measuring point (avoid the logo plates), after zeroing it on the table beside the tote so that table tilt
cancels, and read the tilt across the lid's width. In the model this is the free-edge height u*sin(theta) below 2.0 mm at u = 22.45 cm
from the hinge axis (`SEATED_MM`), which is 0.51 degrees (`SEATED_DEG`): the same limit written as a height, not a second definition.
A lid open 90 degrees or more is never seated. The reading is taken without pressing on the lid (by hand or by a gripper). Why 0.5 degrees: it separates a seated lid (0.0 degrees in the model) from the lid resting on the other
lid's lip (1.4 degrees in the model, 5.5 mm at the measuring point); the inclinometer's accuracy is about +-0.2 degrees (an estimate).
Optional visual check, not scored: when both lids are seated, their top faces are flush across the seam. A real trial is scored on the final settled state.

**Still, clearance and tilt in a real trial.** "Still" is no visible motion of either lid for 2 s of video (the model's limit, a hinge
speed under 0.05 rad/s, moves the lid's free edge 1.1 cm in a second). The 3 cm gripper clearance is measured with a tape measure from
the gripper's tool point (midway between the fingertips) to the closed tote's outline (a box 54.6 x 38.1 cm, up to the top of the seal blocks, 24.1 cm above the table).
The 10 degree tilt limit is read as the change from the starting reading of a two-axis inclinometer or phone level app taped to the tote's end wall (zeroed on the table before the trial).

**Measuring points:** x -9.4 cm from the tote's centre for the left lid and +9.4 cm for the right lid. Each is the middle of the
12.8 cm straight stretch of that lid's edge where it reaches farthest across the centre line (x -15.8 to -3.0 cm on the left lid,
+3.0 to +15.8 cm on the right lid): measure 9.4 cm from the tote's middle toward the arms for the left lid, away from the arms for
the right lid. In the model the lid is rigid, so its angle is the same anywhere on it; the point says where to put the inclinometer, clear of the
logo plates. If a reading of the photos is mirrored (left and right swapped), the feature definition governs, not the x values.

* **1.0:** at the end of the run both lids have been seated, still (hinge speed under 0.05 rad/s), with both gripper tool points
  3 cm or more from the tote's outline, for the last 2 s (40 control steps). The code does not remember a past success: the hold counter resets
  whenever any condition fails, and the run stops as soon as it first reaches 2 s, so the final state is the held state.
* Otherwise, at the end of the run, **only if both grippers are 3 cm or more from the tote** (else 0):
  * **0.75:** both lids are seated (the hold unfinished);
  * **0.5:** one lid is seated and the other is down (open less than 10 degrees) but not seated (as when it rests on the first lid's upper lip: 0.5 degrees or more);
  * **0.25:** exactly one lid is seated (the other open 10 degrees or more), or both are down (open less than 10 degrees) and neither is seated;
  * **0:** every other case (including a gripper on the tote at the end, and lids still lying on the supports).

The tiers rise with progress: 0 < 0.25 < 0.5 < 0.75 < 1.0. `test_scoring.py` checks all combinations.

## What the demo does

Each arm pinches the overhanging near end of its own lid (the lid lies on its support; one pad above and one below the 4 mm
plate, 10 cm out from the hinge), lifts the lid off its support, swings it up past upright and down again to 9 degrees open,
with both arms on the same clock so the lids come down together; lets go, withdraws, and lightly presses the lid with its closed
gripper (13 cm out from the hinge, commanded 3 cm into the plate, held 2 s); then both arms withdraw and go home. Every waypoint is
fixed in advance from the tote's mark and dimensions, and the arm targets are inverse-kinematics solutions checked offline
(worst error 1.3 mm and 0.3 degrees over all 576 steps of each arm). **In a scoring run the demo ends before the press:** the run
ends at step 366, when the 2 s hold completes (lids released at about step 292, seated and still, grippers withdrawn). The plan has
576 steps; the press (its 2 s hold starts at about step 456) and the rest of the plan are not reached in a scoring run.

## Results in simulation (all scores from `task_score`, 1,800-step limit)

| Run | Score | Lids at the end |
|---|---|---|
| Demo, both arms (pinch the overhang, lift off the supports, swing over, lower together, release, press, withdraw), run 1 (320x176) | **1.0** | both 0.0 degrees; held 2 s, success at about step 366 |
| Demo, run 2 (1280x720, the video in `media_local`) | **1.0** | identical trace |
| (i) One lid first: the left (+y) lid is lowered shut, the right arm starts after (run for the full plan, 1,172 steps: the right arm starts 29.8 s, 596 steps, after the left) | 0.5 | left lid 0.0, right lid propped at 1.4 degrees (5.5 mm at the measuring point) on the left lid |
| (ii) Left arm only | 0.25 | left lid 0.0, right lid still lying at 180 on its support |
| (iii) Both lowered together, released, the grippers stay at the lids until the end | 0 | both about 6.2 to 6.5 degrees (held up by the grippers), grippers on the tote |

With the supports present, the lids lie at 180.0 degrees until a gripper moves them, and the tote did not move (0.0 cm at the traces' 0.01 cm resolution, every 20 steps, in all runs).
Not measured: how late a lid may arrive beyond the lip2d.py lead tolerance. The hanging pose (hinge stop) is never reached in a trial with the
supports in place: it is checked offline only (test_scoring.py: no part of a hanging lid overlaps the tub's collision boxes by more than 0.5 mm, and
the lowest point stays above the table).

## Product details (all ESTIMATED, from product photos and the manufacturer's video)

Added so the tote looks like the product. The mechanism, start pose, instruction and scoring do not depend on them, except the lugs' position (set so that a hanging lid clears the rim band) and the bosses (collision boxes that the supports avoid).

| Feature | Size (estimated) | Collision or visual |
|---|---|---|
| Tapered tub walls (nestable) | 0.4 cm thick, outer face leaning in by 5.6 degrees: 2.5 cm in from the rim outline at the base, 0.6 cm in under the rim band | collision (tilted boxes) |
| Rim band | 3.5 cm tall, 1.2 cm thick, outline = the catalogue 54.6 x 38.1 cm, top at 22.9 cm (the lids' rest plane); stands 0.6 cm proud of the wall | collision |
| Slots under the rim band | 2 cm tall, 0.4 cm below the band: two 6 cm wide per long wall, two 5 cm wide per end wall | collision (real gaps in the wall) |
| Recessed panels and ribs | 5 panels per long wall (6 ribs), 2 per end wall (3 ribs, the corner ones shared); ribs 1.4 cm wide, 0.3 cm proud | visual only |
| Foot | ring 0.6 cm tall, 1.2 cm wide at the base footprint, floor 0.8 cm thick above it | collision |
| Hinge knuckles | 7 per side, 4 cm long, 1.2 cm diameter, on each hinge axis | visual only (so they cannot snag the rim; the hinge is still one revolute joint) |
| Seal block hole | 0.6 cm square through each 2 x 2.5 x 0.8 cm block (round in the product) | collision (gap between four boxes) |
| Stacking lugs | 5 x 3 x 0.7 cm at each lid's two outer corners, on the hinge side, 5 to 8 cm from the hinge axis | collision |
| Bosses | two per lid top, 2.4 cm round in the product, 0.3 cm proud; modelled as 2.13 x 2.13 cm squares (same area) at 8 cm from the hinge axis and x +-14 cm | collision |
| Logo plates | embossed, not modelled; kept clear by the supports | none |
| Lid underside ribs | not modelled (the hanging lid in the maker's product image shows a ribbed underside): each lid is a plain 4 mm plate and the grip is on that plate | none |
| Colour | mid blue, one colour for tub and lids | visual |

## Numbers (all ESTIMATED unless marked)

* Catalogue: outside 21.5 x 15 x 9 in, as on the retailer listings the team used (the manufacturer's product page is
  https://akro-mils.com/product/attached-lid-container-39085/). Some listings give the width as 15.25 in; the model uses 15 in.
* Hinge range: 0 to 265 degrees. The manufacturer's video shows an open lid hanging about straight down (about 270 degrees from
  closed); 265 degrees makes the plate lean 5 degrees away from the rim band. A lid hanging at the stop clears the tapered wall, rim band and slots, and its lowest point is
  about 0.5 cm above the table (4.6 mm in the model).
* Seam and lap: see "The seam" above (polyline +-1.5 cm; offset 3.0 cm, 6 cm between levels; 0.2 cm top gap; hidden lap dimensions).
* Seam offset limit: a lid hangs from the hinge at the rim plane, so the lid's top-edge reach (hinge to seam line, 19.45 + 3.0 = 22.45 cm)
  must stay at least 3 mm under the 22.86 cm tote height for the hanging lid to clear the table.
* Supports: part of the task setup, not of the product; dimensions as in the Setup section.
* Everything else: wall, floor and lid thickness; lid width; hinge damping 0.15 N m s/rad (viscous only); masses (tub 1.9 kg,
  lids 0.4 kg each).

## Limits

* The tote is at x 52 cm so that the whole tote fits in the overhead camera frame while both arms reach the lid ends (pinch
  point at x 26 cm).
* The pinch holds a 0.4 kg lid by friction on the lid's near end edge; grip force and press force were not measured.
* The seam geometry (polyline, offset, lap dimensions) is estimated from two photos; which lid overlaps on top in each run is not known.
* The supports are a modelling choice: the real product has no flat open position (manufacturer's video), so a real trial needs the supports as
  specified; the supports carry only the flat part of each lid's top face, clear of the bosses, logo plates, lugs and seal blocks (positions estimated), and the level check confirms each lid lies flat.
* The support top is set to the plate's top face in the model (22.46 cm), 4 mm below the 22.9 cm rim plane; the real plate thickness is estimated.
* The prop angle and the lead tolerance between lids are results of the rigid 2D/3D model with an estimated lap, not product measurements.

## Licence

No third-party assets: the tote is built from procedural boxes and prisms. The robot rig (`yam_arena`, I2RT YAM URDF under MIT) is not
included here.
