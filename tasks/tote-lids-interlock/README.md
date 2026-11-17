# Close an attached-lid tote whose two lids overlap along a stepped seam

**Instruction:** "Close the lids of the tote."

An Akro-Mils 39085 attached-lid container (a blue flip-top tote, 21.5 x 15 x 9 in outside) stands on the table with its long
side along x. Its two lids are hinged on the long sides and lie open, flat and outward, on their hinge stops. The robot
(both YAM arms, parallel grippers) has to shut both lids, flat and seated on the rim, withdraw both grippers, and leave the
tote on its mark.

The lids meet along a stepped seam (like a Z): in the -x half of their length the +y lid is the wider one, in the +x half the
-y lid is, with a 45 degree jog between. In each half the wider lid carries a raised flange that laps over the other lid's low
tongue. There is no snap or latch: a lid shut on the rim stays there by its weight. In the simulator's rigid model, closing one
lid first leaves the second propped open about 6.7 degrees on the first lid's flange, so the lids have to come down together.

## Evidence for the seam and closing behaviour

On this tote the lids cannot be closed one at a time: each lid laps over the other along the seam, so a lid shut first
blocks the second. They have to come down together, as in a seller's product video at 0:19
(https://www.amazon.ca/vdp/09c3c1c46a40487390129b9b26503d1c), where the two lids are closed together and each lid has
overlap geometry along the seam. The simulation reproduces this: a lid shut first props the second at 6.7 degrees. The seam
offset, flange height, gaps and which lid overlaps on top in each half are estimates.

## Why it matters

Attached-lid totes are a standard container for order picking and distribution (manufacturer and reseller product pages).
Stockers and order fillers (O*NET 53-7065.00; 2,804,300 US employees in 2025, BLS data as shown on O*NET) fill customers'
orders from warehouse stock, and their O*NET core tasks include packing and unpacking items in warehouses and issuing
merchandise. Hand packers and packagers (O*NET 53-7064.00; 555,500 employees in 2025) seal containers. Closing a tote's
lids is part of that handling, not a separately listed task in either occupation.

## Files

| File | What |
|---|---|
| `task.py` | the task file: scene, scoring, `demo()` |
| `totekit.py` | the tote as one articulation (tub root, two revolute lid hinges, cuboid colliders for the tapered walls, rim band, foot, every plate, flange, tongue, jog step, seal block and lug; visual-only ribs, knuckles and bosses) and the hooks yam_arena needs to spawn it |
| `demo.py` | the scripted two-arm plan (open loop) and the control-run modes |
| `lip2d.py` | offline 2D cross-section check used to choose the lip dimensions |
| `test_scoring.py` | offline tests of scoring and geometry |
| `task.json` | the submission fields |

## Run and score a trial

On a machine with the `yam_arena` rig (Isaac Sim, Isaac Lab-Arena 0.3.1), from this folder:

    inspect-robots run --embodiment yam_arena -E task=task.py -E scene=station -E object_jitter=0 -E control_hz=20 \
      -E cam_width=320 -E cam_height=176 --instruction "Close the lids of the tote." \
      --scorer task_score --max-steps 1800 --store-frames --log-dir runs/trial

Scripted demo (open loop, from the fixed layout; no simulator state is read, nothing attached, no helper forces):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E scene=station -E object_jitter=0 \
      -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "Close the lids of the tote." \
      --scorer task_score --max-steps 1300 --store-frames --log-dir runs/demo
    inspect-robots video runs/demo/adhoc*.json

`task_score` reports 1.0 on success, otherwise the partial credit. Environment variables for control runs:
`TOTE_MODE=both|nopress|stay|seq|left|right`, `TOTE_LAG=<seconds>` (holds the right arm back), `TOTE_TRACE=1` (prints lid
angles and tote drift every 20 steps). Unit tests: `python -m pytest test_scoring.py` with `yam_arena` importable.

**Dream-1 run: 0.0** (1,800 steps; it never closed a lid).

**Runtime per trial:** the demo reaches full score at about step 360 (18 s of simulated time at 20 Hz); the one-lid-first control runs about 1,200 steps. Wall time per trial was not timed. **Seeds:** none; `-E object_jitter=0` fixes the layout.

## Scoring (also in `task.json`)

0 for the whole trial if the tote ever slides more than 3 cm from its mark or tilts 10 degrees or more.

**Measuring point (the same in the code and by hand):** a lid is seated when the height of its free (seam-side) plate edge
above the rim plane, at the middle of its straight seam section, is under 3 mm and the lid is on the tote. The point is at
the middle of the straight section where that lid is the wider one: x = -13.9 cm for the +y lid, +13.9 cm for the -y lid. By
hand: lay a straightedge across the rim there and measure the gap under the lid's edge with a ruler. In code this is
`edge_height = u * sin(theta)` from the hinge angle (u is the distance from the hinge axis to that edge).

* **1.0:** both lids seated, still (hinge speed under 0.05 rad/s), and both gripper tool points 3 cm or more from the tote's
  outline, held 2 s.
* Otherwise, at the end of the run, **only if both grippers are 3 cm or more from the tote** (else 0):
  * **0.5:** one lid is seated and the other is propped on its lap (open less than 10 degrees);
  * **0.25:** exactly one lid is seated, or both are down (open less than 10 degrees) without the hold being completed;
  * **0:** every other case (including both lids seated with a gripper on the tote at the end).

Every outcome is covered by one of these; `test_scoring.py` checks all combinations. Real world: tape measure for the slide,
straightedge and ruler for the 3 mm, a phone level for the 10 degree tilt and open angles.

## What the demo does

Each arm pinches the near end of its own lid between its fingers (one pad above, one below the 4 mm plate), swings the lid up
past upright and down again to 9 degrees open, with both arms on the same clock so the lids come down together; lets go,
withdraws, and, if needed for the lid to sit flat, lightly presses the lid's edge with its closed fingers (commanded 3 cm into
the plate, held 2 s); then both arms withdraw and go home.

## Results in simulation (all scores from `task_score`)

| Run | Score | Lids at the end |
|---|---|---|
| Demo, both arms (pinch, lower together, release, withdraw; the press is not needed) | **1.0** (2 of 2 runs, one at 1280x720 for the media) | both 0.0 degrees; the lids settle by their weight, still by step 340 |
| (i) One lid first: the +y lid pinched, lowered and pressed shut, the -y arm starts after | 0.5 | +y lid 0.0, -y lid propped at 6.7 degrees on its flange |
| (ii) Left arm only | 0.25 | left lid 0.0, right lid open at 180 |
| (iii) Both lowered together, released, the fingers stay at the lids until the end | 0 | both about 6.2 to 6.4 degrees (held up by the fingers), grippers on the tote |

The one-lid-first jam in the simulator (6.7 degrees) matches the 2D cross-section check (`lip2d.py`: 6.7 degrees in either
order, in both halves of the seam). Only the +y-lid-first order was run in the simulator. The `nopress` mode (no press at all)
also scores 1.0, since the demo's press comes after the lids have already settled; the press is kept in the plan only as a
light fallback. The tote moved at most 0.1 cm in these runs. Not measured: how late a lid may arrive.

## Product-photo details (all ESTIMATED from a verbal description of a product photo)

Added so the tote looks like the product. Mechanism, start pose, instruction and scoring do not depend on them.

| Feature | Size (estimated) | Collision or visual |
|---|---|---|
| Tapered tub walls (nestable) | 0.4 cm thick, outer face leaning in by 5.6 degrees: 2.5 cm in from the rim outline at the base, 0.6 cm in under the rim band | collision (tilted boxes) |
| Rim band | 3.5 cm tall, 1.2 cm thick, outline = the catalogue 54.6 x 38.1 cm, top at 22.9 cm (the lids' rest plane, as before); stands 0.6 cm proud of the wall | collision |
| Slots under the rim band | 2 cm tall, 0.4 cm below the band: two 6 cm wide per long wall, two 5 cm wide per end wall | collision (real gaps in the wall) |
| Recessed panels and ribs | 5 panels per long wall (6 ribs), 2 per end wall (3 ribs, the corner ones shared); ribs 1.4 cm wide, 0.3 cm proud | visual only (the panel recess is shown as ribs proud of the wall) |
| Foot | ring 0.6 cm tall, 1.2 cm wide at the base footprint, floor 0.8 cm thick above it | collision |
| Hinge knuckles | 7 per side, 4 cm long, 1.2 cm diameter, on each hinge axis | visual only (so they cannot snag the rim; the hinge is still one revolute joint) |
| Seal block hole | 0.6 cm square through each 2 x 2.5 x 0.8 cm block (round in the product) | collision (gap between four boxes) |
| Stacking lugs | 5 x 3 x 0.7 cm at each lid's two outer corners, on the hinge side | collision |
| Bosses | two round, 2.4 cm diameter, 0.3 cm high per lid top | visual only |
| Colour | mid blue, one colour for tub and lids | visual |

Lugs start 1.6 cm from the hinge axis and 20 cm from the lid's middle, so they sit clear of the demo's pinch (10 cm out, at
the lid's end) and press points (15.5 cm out). The embossed lettering and logos are not modelled. The hand-hold openings
are the slots. The tub's body is still a single rigid root; the lid's rest height and hinge position did not change.

Lugs start 1.6 cm from the hinge axis and 20 cm from the lid's middle, so they sit clear of the demo's pinch (10 cm out, at
the lid's end) and press points (15.5 cm out). The embossed lettering and logos are not modelled. The hand-hold openings are
the slots. The tub's body is a single rigid root.

## Numbers (all ESTIMATED unless marked)

* Catalogue: outside 21.5 x 15 x 9 in (manufacturer product page https://akro-mils.com/product/attached-lid-container-39085/
  and sell sheet https://akro-mils.com/Resources/Sell-Sheets/Attached-Lid-Containers-SS-1.pdf, listed by search but not opened
  here; retailer listings such as Zoro and Industrial Shelving agree on 21.5 x 9 in). Some retailers list the width as 15.25 in;
  the model uses 15 in.
* Seam and lap (estimated from a description of a product photo): stepped seam, offset 1.75 cm each side of the centre line
  (about 9 % of a lid's width), 45 degree jog 1.7 cm long, built as a four-step staircase of plain low plate; flange underside
  16 mm above the plate's lower face on a 4 mm riser, reaching 8 mm past the seam; the other lid's tongue ends 2 mm short of
  the riser; the lids' plain edges stop 1.0 cm from the seam line. A 2 x 2.5 x 0.8 cm raised seal block at each lid's seam
  ends, with a 6 mm square hole through it (a gap between four boxes); it has no function. Hinge knuckles are visual only (one
  hinge joint per lid); the lids overhang the rim by the 4 mm between the rim's outer face and the hinge axis.
* Everything else: wall, floor and lid thickness; lid width; hinge stop at 180 degrees; hinge damping 0.15 N m s/rad (viscous
  only); masses (tub 1.9 kg, lids 0.4 kg each).
* The lids start open flat (180 degrees) because the arms cannot reach a hanging lid; a real tote's lids usually hang.

## Limits

* The tote is at x 52 cm so that the whole tote fits in the overhead camera frame while both arms reach the lid ends (pinch
  point at x 26 cm).
* The pinch holds a 0.4 kg lid by friction on the lid's near end edge; grip force and press force were not measured.
* The seam geometry follows a description of one photo; which lid overlaps on top in each half and the flange heights are not
  confirmed.
* The jam angle and the lead tolerance between lids (in the 2D cross-section check either lid may lead the other by up to 2 degrees while both come down) are results of the rigid 2D/3D model, not product measurements.

## Licence

No third-party assets: the tote is built from procedural boxes. The robot rig (`yam_arena`, I2RT YAM URDF under MIT) is not
included here.
