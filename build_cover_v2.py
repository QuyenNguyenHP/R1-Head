"""Camera cover v2 (D415) for R1 head - hangs on the 2 ORIGINAL M3 screws (no new screws).

Loosen each original screw ~4 mm, lower the cover from above so the U-slots slide over the
screw shanks, then tighten the same screws. The head of each screw clamps a ~3 mm plate that
sits on the flat seat (spot face) around the hole. Tape over the top pad for extra stability.

Coordinates = head_yaw_link.STL frame, in mm: +X forward, +Y left, +Z up.
Run:  python build_cover_v2.py   -> writes d415_cover_v2.stl (+ prints checks)
Needs: pip install numpy trimesh manifold3d
"""
import numpy as np
import trimesh
import manifold3d as mf

# ---------------- parameters ----------------
CAM_L, CAM_H, CAM_D = 99.0, 23.0, 20.0     # D415 size (Y, Z, X)
CAM_Z0 = 8.0                               # camera bottom height (Z)
CAM_BACK_X = 91.0                          # camera back face (X)
CLR = 0.4                                  # clearance around camera
WALL = 2.5                                 # shell wall
BACK_WALL = 2.7                            # wall behind camera (M3 screws go through)
POCKET_R = 4.0                             # pocket corner radius (front view)
CAM_M3_Y, CAM_M3_D = 22.5, 3.4             # D415 back M3 holes: +-22.5 (45 mm pitch)

BOLT_Y, BOLT_Z = 47.3, -0.5                # existing head screws (centre)
SEAT_X = 59.1                              # flat seat (spot face) under the original screw head
PLATE_T = 3.5                              # seat->clamp face; screw must be backed out ~3.5 mm (you have ~4)
SLIDE = 10.0                               # cover is lowered this far onto the screws
CLAMP_FACE_X = SEAT_X + PLATE_T            # where the screw head presses
SCREW_HEAD_D = 7.05                        # pocket for M3 socket head (5.5) + key access
M3_SLOT = 3.65                             # U-slot width for the M3 shank
BLK_DY, BLK_Z = (-6.0, 6.5), (-4.0, 8.0)   # clamp block extent around each screw (dy toward outside +)
BLK_FRONT_X = 100.0
BLK_GRID = 0.5
PAD_Y, PAD_Z = (-36.0, 36.0), (28.0, 40.0) # forehead rest pad region
PAD_GAP = 0.4
GRID = 1.0

SEG = 48
# --------------------------------------------

cam_cy = CAM_Z0 + CAM_H / 2
cam_front_x = CAM_BACK_X + CAM_D


def yz_prism(cs, x0, x1):
    """Extrude a CrossSection drawn in (Y,Z) along X from x0 to x1."""
    m = mf.Manifold.extrude(cs, x1 - x0)
    # (u,v,w) -> (x=w+x0, y=u, z=v)  (cyclic permutation keeps orientation)
    return m.warp_batch(lambda v: np.c_[v[:, 2] + x0, v[:, 0], v[:, 1]])


def rrect(w, h, r, cy, cz=None):
    if cz is None:
        cy, cz = 0.0, cy
    r = min(r, w / 2 - 1e-3, h / 2 - 1e-3)
    pts = []
    for sx, sz in [(1, 1), (-1, 1), (-1, -1), (1, -1)]:
        c = mf.CrossSection.circle(r, SEG).translate((cy + sx * (w / 2 - r), cz + sz * (h / 2 - r)))
        pts.append(c)
    return mf.CrossSection.batch_hull(pts)


def cyl_x(d, x0, x1, y, z):
    c = mf.Manifold.cylinder(x1 - x0, d / 2, d / 2, SEG)          # along +Z
    c = c.warp_batch(lambda v: np.c_[v[:, 2] + x0, v[:, 0] + y, v[:, 1] + z])
    return c


def box(x0, x1, y0, y1, z0, z1):
    return mf.Manifold.cube((x1 - x0, y1 - y0, z1 - z0)).translate((x0, y0, z0))


# ---- shell around camera ----
pw, ph = CAM_L + 2 * CLR, CAM_H + 2 * CLR
outer = yz_prism(rrect(pw + 2 * WALL, ph + 2 * WALL, POCKET_R + WALL, cam_cy),
                 CAM_BACK_X - CLR - BACK_WALL, cam_front_x + 0.5)
pocket = yz_prism(rrect(pw, ph, POCKET_R, cam_cy), CAM_BACK_X - CLR, cam_front_x + 5)
body = outer - pocket

# camera M3 holes + vents in back wall
bw0, bw1 = CAM_BACK_X - CLR - BACK_WALL - 1, CAM_BACK_X
for s in (-1, 1):
    body -= cyl_x(CAM_M3_D, bw0, bw1, s * CAM_M3_Y, cam_cy)
body -= yz_prism(rrect(28, 13, 3, cam_cy), bw0, bw1)                       # centre vent
for s in (-1, 1):
    body -= yz_prism(rrect(12, 13, 3, s * 38.0, cam_cy), bw0, bw1)         # outer vents
# USB-C / cable slots on both ends
for s in (-1, 1):
    y0 = s * (pw / 2 - 1)
    body -= box(CAM_BACK_X + 2, cam_front_x - 2, min(y0, s * 60), max(y0, s * 60),
                cam_cy - 5.5, cam_cy + 5.5)

# ---- forehead rest pad (follows the real forehead surface) ----
head = trimesh.load('head_yaw_link.STL')
head.apply_scale(1000)
pts, _ = trimesh.sample.sample_surface(head, 400000, seed=1)
front = pts[pts[:, 0] > 40]


def heightfield_solid(ys, zs, XB, XF):
    """Closed solid between back surface XB(y,z) and front surface XF(y,z) on a grid."""
    YY, ZZ = np.meshgrid(ys, zs, indexing='ij')
    nY, nZ = XB.shape
    V = np.vstack([np.c_[XB.ravel(), YY.ravel(), ZZ.ravel()], np.c_[XF.ravel(), YY.ravel(), ZZ.ravel()]])
    off = nY * nZ
    idx = lambda i, j: i * nZ + j
    F = []
    for i in range(nY - 1):
        for j in range(nZ - 1):
            a, b, c, d = idx(i, j), idx(i + 1, j), idx(i + 1, j + 1), idx(i, j + 1)
            F += [[a, c, b], [a, d, c], [a + off, b + off, c + off], [a + off, c + off, d + off]]
    border = [idx(i, 0) for i in range(nY)] + [idx(nY - 1, j) for j in range(1, nZ)] +              [idx(i, nZ - 1) for i in range(nY - 2, -1, -1)] + [idx(0, j) for j in range(nZ - 2, 0, -1)]
    for k in range(len(border)):
        p, q = border[k], border[(k + 1) % len(border)]
        F += [[p, q, q + off], [p, q + off, p + off]]
    t = trimesh.Trimesh(V, np.array(F), process=True)
    if t.volume < 0:
        t.invert()
    m = mf.Manifold(mf.Mesh(vert_properties=np.asarray(t.vertices, np.float32),
                            tri_verts=np.asarray(t.faces, np.uint32)))
    assert m.status() == mf.Error.NoError and m.volume() > 0, m.status()
    return m


def surface_x(yc, zc, r=0.9, fallback=None):
    """Highest head surface over the slide path (cover comes down from SLIDE mm above)."""
    q = front[(abs(front[:, 1] - yc) < r) & (front[:, 2] > zc - r) & (front[:, 2] < zc + SLIDE + r)]
    return q[:, 0].max() if len(q) else fallback


# ---- clamp blocks on the original screws (U-slot open downward) ----
for s_ in (-1, 1):
    yb = s_ * BOLT_Y
    dys = np.arange(BLK_DY[0], BLK_DY[1] + 1e-6, BLK_GRID)
    zs_ = np.arange(BLK_Z[0], BLK_Z[1] + 1e-6, BLK_GRID)
    ys_ = yb + s_ * dys
    if s_ < 0:
        ys_ = ys_[::-1]
    XB = np.array([[surface_x(y_, z_, fallback=SEAT_X) + PAD_GAP for z_ in zs_] for y_ in ys_])
    # never reach into the screw hole: near the axis stay in front of the seat
    YYb, ZZb = np.meshgrid(ys_, zs_, indexing='ij')
    near_axis = np.hypot(YYb - yb, ZZb - BOLT_Z) < SCREW_HEAD_D / 2 + 1.0
    XB[near_axis] = np.maximum(XB[near_axis], SEAT_X + PAD_GAP)
    XF = np.maximum(BLK_FRONT_X, XB + 1.0)
    blk = heightfield_solid(ys_, zs_, XB, XF)
    # tie block to the camera shell
    blk = blk + box(80, cam_front_x - 4, yb - 6, yb + 6, 4.0, CAM_Z0)
    body = body + blk
    # shank slot + head pocket, both open downward, head pocket open forward for the hex key
    body -= cyl_x(M3_SLOT, 40, CLAMP_FACE_X + 0.01, yb, BOLT_Z)
    body -= box(40, CLAMP_FACE_X + 0.01, yb - M3_SLOT / 2, yb + M3_SLOT / 2, BOLT_Z - 30, BOLT_Z + 0.23)
    body -= cyl_x(SCREW_HEAD_D, CLAMP_FACE_X, 200, yb, BOLT_Z)
    body -= box(CLAMP_FACE_X, 200, yb - SCREW_HEAD_D / 2, yb + SCREW_HEAD_D / 2, BOLT_Z - 30, BOLT_Z + 0.23)

x_back_shell = CAM_BACK_X - CLR - BACK_WALL
shell_top = cam_cy + ph / 2 + WALL
ys = np.arange(PAD_Y[0], PAD_Y[1] + 1e-6, GRID)
zs = np.arange(PAD_Z[0], PAD_Z[1] + 1e-6, GRID)
XB = np.zeros((len(ys), len(zs)))
XF = np.zeros_like(XB)
for i, yc in enumerate(ys):
    for j, zc in enumerate(zs):
        q = front[(abs(front[:, 1] - yc) < 1.2) & (front[:, 2] > zc - 1.2) & (front[:, 2] < zc + SLIDE + 1.2)]
        XB[i, j] = q[:, 0].max() + PAD_GAP              # conservative: highest point nearby
        x_end = x_back_shell + 1.0
        if zc > shell_top - 2:                          # chamfer the part above the shell
            x_end -= (zc - (shell_top - 2)) * 1.2
        XF[i, j] = max(x_end, XB[i, j] + 1.0)
YY, ZZ = np.meshgrid(ys, zs, indexing='ij')
nY, nZ = XB.shape
vb = np.c_[XB.ravel(), YY.ravel(), ZZ.ravel()]
vf = np.c_[XF.ravel(), YY.ravel(), ZZ.ravel()]
V = np.vstack([vb, vf])
off = nY * nZ
idx = lambda i, j: i * nZ + j
F = []
for i in range(nY - 1):
    for j in range(nZ - 1):
        a, b, c, d = idx(i, j), idx(i + 1, j), idx(i + 1, j + 1), idx(i, j + 1)
        F += [[a, c, b], [a, d, c]]                                  # back surface
        F += [[a + off, b + off, c + off], [a + off, c + off, d + off]]  # front surface
border = [idx(i, 0) for i in range(nY)] + [idx(nY - 1, j) for j in range(1, nZ)] + \
         [idx(i, nZ - 1) for i in range(nY - 2, -1, -1)] + [idx(0, j) for j in range(nZ - 2, 0, -1)]
for k in range(len(border)):
    p, q = border[k], border[(k + 1) % len(border)]
    F += [[p, q, q + off], [p, q + off, p + off]]
padm = trimesh.Trimesh(V, np.array(F), process=True)
if padm.volume < 0:
    padm.invert()
pad = mf.Manifold(mf.Mesh(vert_properties=np.asarray(padm.vertices, np.float32),
                          tri_verts=np.asarray(padm.faces, np.uint32)))
assert pad.status() == mf.Error.NoError and pad.volume() > 0, pad.status()
cols = [pad]
body = body + pad

# ---- export ----
mesh = body.to_mesh()
out = trimesh.Trimesh(np.asarray(mesh.vert_properties)[:, :3], np.asarray(mesh.tri_verts), process=False)
out.export('d415_cover_v2.stl')
print('cover bbox', out.bounds.round(1), 'volume cm3', round(out.volume / 1000, 1),
      'watertight', out.is_watertight)


# ---- collision check: head surface points inside the cover ----
def winding(points, tri):
    w = np.zeros(len(points))
    for i in range(0, len(points), 400):
        P = points[i:i + 400][:, None, None, :]
        A = tri[None] - P
        a, b, c = A[..., 0, :], A[..., 1, :], A[..., 2, :]
        la, lb, lc = (np.linalg.norm(x, axis=-1) for x in (a, b, c))
        det = np.einsum('ijk,ijk->ij', a, np.cross(b, c))
        den = la * lb * lc + np.einsum('ijk,ijk->ij', a, b) * lc + \
            np.einsum('ijk,ijk->ij', b, c) * la + np.einsum('ijk,ijk->ij', c, a) * lb
        w[i:i + 400] = (2 * np.arctan2(det, den)).sum(1) / (4 * np.pi)
    return w

lo, hi = out.bounds
near = pts[np.all((pts > lo - 0.1) & (pts < hi + 0.1), axis=1)]
inside = near[winding(near, out.triangles) > 0.5]
print('head points tested', len(near), 'inside cover', len(inside))
if len(inside):
    print('  collision bbox', inside.min(0).round(1), inside.max(0).round(1))
np.save('_collisions_v2.npy', inside)

# ---- assembly path: cover is lowered onto the screws from above ----
for up in (2, 4, 6, 8, 10):
    moved = out.copy()
    moved.apply_translation([0, 0, up])
    lo2, hi2 = moved.bounds
    near2 = pts[np.all((pts > lo2 - 0.1) & (pts < hi2 + 0.1), axis=1)]
    n_in = int((winding(near2, moved.triangles) > 0.5).sum())
    print(f'cover raised {up:2d} mm: head points inside = {n_in}')

# clamp plate thickness actually under the screw head (annulus r 1.8..2.75)
for s_ in (-1, 1):
    ring = [surface_x(s_ * BOLT_Y + 2.3 * np.cos(a), BOLT_Z + 2.3 * np.sin(a), r=0.3, fallback=SEAT_X) + PAD_GAP
            for a in np.linspace(0, 2 * np.pi, 24, endpoint=False)]
    ring = np.maximum(ring, SEAT_X + PAD_GAP)
    covered = np.mean(np.array(ring) < CLAMP_FACE_X - 1.0)
    print(f'side {s_:+d}: head contact on {covered*100:.0f}% of the ring, plate {CLAMP_FACE_X-max(min(ring),SEAT_X):.1f} mm thick at thinnest contact')
