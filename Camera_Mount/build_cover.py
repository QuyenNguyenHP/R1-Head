"""Camera cover (D415) for R1 head - bolts onto the 2 existing M3 screws on the forehead.

Coordinates = head_yaw_link.STL frame, in mm: +X forward, +Y left, +Z up.
Run:  python build_cover.py      -> writes d415_cover.stl (+ prints checks)
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
BOLT_FLOOR_X = 59.4                        # seat of the old M3 screw head in the head counterbore
NOSE_D, NOSE_TO_X = 5.6, 65.5              # centring nose that goes into the counterbore
STANDOFF_D = 7.0                           # main tube (stays in front of the counterbore)
TAB_FRONT_X, TAB_BACK_X = 92.0, 84.0
TAB_Z0, TAB_Z1 = -4.0, 8.0
TAB_W = 12.0
HEAD_CB_D, HEAD_CB_DEPTH = 6.2, 3.5        # recess for the new M3 socket head
M3_CLEAR = 3.4

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

# ---- tabs + standoffs onto the existing head screws ----
for s in (-1, 1):
    y = s * BOLT_Y
    tab = box(TAB_BACK_X, TAB_FRONT_X, y - TAB_W / 2, y + TAB_W / 2, TAB_Z0, TAB_Z1)
    # tie tab into the shell bottom
    tie = box(TAB_BACK_X, cam_front_x - 4, y - TAB_W / 2, y + TAB_W / 2, TAB_Z1 - 4, CAM_Z0)
    st = cyl_x(NOSE_D, BOLT_FLOOR_X, NOSE_TO_X + 0.5, y, BOLT_Z) + \
         cyl_x(STANDOFF_D, NOSE_TO_X, TAB_BACK_X + 1, y, BOLT_Z)
    body = body + tab + tie + st
    body -= cyl_x(M3_CLEAR, BOLT_FLOOR_X - 1, TAB_FRONT_X + 1, y, BOLT_Z)
    body -= cyl_x(HEAD_CB_D, TAB_FRONT_X - HEAD_CB_DEPTH, cam_front_x + 30, y, BOLT_Z)

# ---- forehead rest pad (follows the real forehead surface) ----
head = trimesh.load('head_yaw_link.STL')
head.apply_scale(1000)
pts, _ = trimesh.sample.sample_surface(head, 400000, seed=1)
front = pts[pts[:, 0] > 40]
x_back_shell = CAM_BACK_X - CLR - BACK_WALL
shell_top = cam_cy + ph / 2 + WALL
ys = np.arange(PAD_Y[0], PAD_Y[1] + 1e-6, GRID)
zs = np.arange(PAD_Z[0], PAD_Z[1] + 1e-6, GRID)
XB = np.zeros((len(ys), len(zs)))
XF = np.zeros_like(XB)
for i, yc in enumerate(ys):
    for j, zc in enumerate(zs):
        q = front[(abs(front[:, 1] - yc) < 1.2) & (abs(front[:, 2] - zc) < 1.2)]
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
out = trimesh.Trimesh(np.asarray(mesh.vert_properties)[:, :3], np.asarray(mesh.tri_verts))
out.export('d415_cover.stl')
print('cover bbox', out.bounds.round(1), 'volume cm3', round(out.volume / 1000, 1),
      'watertight', out.is_watertight)
print('pad columns', len(cols))

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
np.save('_collisions.npy', inside)
