"""2D cross-section (y,z) check of the lids' overlap (offline, numpy only): which (thetaA, thetaB) pairs collide.

The seam is stepped: in the -x half the +y lid ("A") is wider and carries a raised flange over the -y lid's low tongue; in the +x
half the roles swap. Each half's cross-section is checked (the lids are rigid, so both must be free of contact). Used to
check the overlap in totekit.py: python lip2d.py prints the jam angle and the lead tolerance."""
import math
import numpy as np

H = 0.2286; WIDTH = 0.381; HY = WIDTH / 2 + 0.004; T = 0.004; C = 0.010; RT = 0.004; OV = 0.008; PS = 0.002
D = 0.0175          # seam offset
L_DEFAULT = 0.016   # flange level


def rects(side, wide, yc, L):
    """Rectangles (u0, u1, w0, w1) in the lid's frame: u from the hinge axis, w off the lower face."""
    if wide:
        ue = HY - side * yc - C
        return [(0, ue, 0, T), (ue - RT, ue, 0, L + T), (ue - RT, HY - side * yc + OV, L, L + T)]
    return [(0, HY - side * yc + C - PS, 0, T)]


def poly(side, theta, rc):
    d = np.array([-side * math.cos(theta), math.sin(theta)])
    n = np.array([-side * math.sin(theta), math.cos(theta)])
    o = np.array([side * HY, H])
    out = []
    for u0, u1, w0, w1 in rc:
        pts = [o + u * d + w * n for u in (u0, u1) for w in (w0, w1)]
        out.append(np.array([pts[0], pts[1], pts[3], pts[2]]))
    return out


def sat(p, q):
    for poly_ in (p, q):
        for i in range(4):
            e = poly_[(i + 1) % 4] - poly_[i]
            ax = np.array([-e[1], e[0]])
            if np.linalg.norm(ax) < 1e-12:
                continue
            ax /= np.linalg.norm(ax)
            a = [ax @ v for v in p]
            b = [ax @ v for v in q]
            if max(a) <= min(b) + 1e-5 or max(b) <= min(a) + 1e-5:
                return False
    return True


def collide(ta, tb, L=L_DEFAULT, d=None):
    """Whether the +y lid at ta degrees and the -y lid at tb degrees touch in either half of the seam."""
    d = D if d is None else d
    ta, tb = math.radians(ta), math.radians(tb)
    for yc, wide_side in ((-d, +1), (+d, -1)):
        A = poly(+1, ta, rects(+1, wide_side == +1, yc, L))
        B = poly(-1, tb, rects(-1, wide_side == -1, yc, L))
        if any(sat(a, b) for a in A for b in B):
            return True
    return False


def jam_angle(L=L_DEFAULT, d=None):
    """Opening of the second lid where it stops against the first, shut flat (None if it closes)."""
    if collide(0, 0, L, d):
        return float("nan")
    for tb in np.arange(40, -0.01, -0.1):
        if collide(0, tb, L, d):
            return round(float(tb), 1)
    return None


if __name__ == "__main__":
    print("flat-flat collide:", collide(0, 0))
    print("one lid shut first, the other jams at", jam_angle(), "deg (+y lid shut)")
    print("one lid shut first, the other jams at", next((round(float(a), 1) for a in np.arange(40, -0.01, -0.1) if collide(a, 0)), None), "deg (-y lid shut)")
    for lag in (0, 0.5, 1, 2, 3, 4, 6):
        ok = all(not collide(ta, max(ta - lag, 0)) for ta in np.arange(40, -0.01, -0.2))
        ok2 = all(not collide(max(ta - lag, 0), ta) for ta in np.arange(40, -0.01, -0.2))
        print(f"lead {lag} deg: +y leads {'free' if ok else 'blocked'}, -y leads {'free' if ok2 else 'blocked'}")
