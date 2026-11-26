"""
Gripper SO101 đồng trục với cánh tay Unitree R1 — script tham số (Python + manifold3d + trimesh).

Ý tưởng
  - Bàn tay R1 (right_wrist_roll_link.STL) là nắm đấm có lỗ trụ xuyên Ø30 theo trục Z của link.
  - Một KHUNG CHỮ U ôm nắm tay:
      * tấm trên + tay cầm Ø29.2 (cắm vào lỗ) + tấm trước (đứng trước đốt ngón tay) -> in liền 1 khối
      * tấm dưới (rời) siết bằng 1 bu lông M5 vào tay cầm + 2 vít M4 vào chân tấm trước
  - Gripper SO101 (wrist_roll_follower_so101_v1 + STS3215 + moving_jaw) bắt vào mặt trước tấm trước,
    hướng theo +X (dọc cẳng tay). Điểm 2 mũi kẹp chạm nhau nằm ĐÚNG trên trục cánh tay (trục X của link,
    tức trục khớp wrist roll), nên khi xoay cổ tay mũi kẹp chỉ xoay tại chỗ.
  - Hàm kẹp mở lên/xuống: hàm cố định ở dưới, hàm di động (motor) ở trên.

Hệ tọa độ: hệ link right_wrist_roll_link của R1, đơn vị mm.
Chạy:  python make_r1_gripper.py      -> kết quả trong output/
"""
import os, sys
import numpy as np
import trimesh
import manifold3d as m3d

sys.stdout.reconfigure(encoding="utf-8")
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "output")
os.makedirs(OUT, exist_ok=True)

# ================= SỐ ĐO TAY R1 (từ STL) =================
HOLE_XY = (110.53, 15.01)   # tâm lỗ nắm tay (trục lỗ // Z)
FIST_TOP = 34.4             # mặt trên nắm tay (phía ngón cái) quanh khu vực khung
FIST_BOT = -31.1            # mặt dưới (phía ngón út)
FIST_FRONT = 142.7          # điểm xa nhất phía trước (đốt ngón tay)
ARM_AXIS_YZ = (0.0, 0.0)    # trục cánh tay = trục X của link (trục khớp wrist roll)

# ================= SỐ ĐO SO101 follower (hệ follower) =================
# mặt lắp horn z_f=0; 4 lỗ M3 tại (±4.95, -0.22±4.95); hàm cố định chạy theo +z_f
HORN_CENTER_F = (0.0, -0.22)
HORN_HOLE_OFF = 4.95
TIP_F = (-8.5, 0.0, 102.0)  # điểm mũi hàm cố định (mặt kẹp trong, gần đầu mũi)

# ================= THAM SỐ THIẾT KẾ =================
HANDLE_D = 29.2       # chỉnh theo kết quả test plug
CLAMP_GAP = 2.5       # tay cầm ngắn hơn nắm tay -> tấm dưới ép chặt được
TIP_CHAMFER = 1.2
PLATE_T = 6.0         # tấm trên / tấm dưới
FRONT_GAP = 2.5       # khe giữa đốt ngón tay và tấm trước
FRONT_T = 10.0        # dày tấm trước (đủ cho vít M3 bắt gripper và vít M4 ở chân)
FRONT_HALF_W = 25.0   # nửa bề rộng tấm trước (theo Y)
COLLAR_R = 19.0       # bán kính phần tấm bao quanh lỗ
TIP_SHIFT_Z = 0.0     # dịch điểm mũi kẹp lên/xuống so với trục (0 = đúng trục)

M3_PILOT_D, M3_PILOT_DEPTH = 2.6, 8.0     # vít M3 tự ren bắt follower
M4_PILOT_D, M4_PILOT_DEPTH = 3.4, 14.0    # vít M4 tự ren ở chân tấm trước
M4_CLEAR_D, M4_HEAD_D, M4_HEAD_T = 4.5, 8.0, 3.0
M4_Y = 15.0
M5_CLEAR = 5.5
M5_NUT_AF, M5_NUT_T = 8.4, 4.6
NUT_SLOT_FROM_END = 15.0
BOLT_HOLE_DEPTH = 32.0
M5_HEAD_D, M5_HEAD_T = 9.5, 3.0
SEG = 128

# ================= tiện ích =================
def cyl(h, d, z0=0.0, d2=None, seg=SEG):
    return m3d.Manifold.cylinder(h, d / 2, (d2 if d2 is not None else d) / 2, seg).translate((0, 0, z0))

def box(x0, x1, y0, y1, z0, z1):
    return m3d.Manifold.cube((x1 - x0, y1 - y0, z1 - z0)).translate((x0, y0, z0))

def hex_prism(af, h, z0):
    r = af / np.sqrt(3)
    pts = [(r * np.cos(np.pi / 3 * i), r * np.sin(np.pi / 3 * i)) for i in range(6)]
    return m3d.Manifold.extrude(m3d.CrossSection([pts]), h).translate((0, 0, z0))

def to_tm(man):
    mesh = man.to_mesh()
    return trimesh.Trimesh(np.asarray(mesh.vert_properties)[:, :3], np.asarray(mesh.tri_verts), process=False)

def from_tm(tm):
    man = m3d.Manifold(m3d.Mesh(vert_properties=np.asarray(tm.vertices, np.float32),
                                tri_verts=np.asarray(tm.faces, np.uint32)))
    assert man.status() == m3d.Error.NoError, man.status()
    return man

def save(man, name):
    tm = to_tm(man)
    tm.export(os.path.join(OUT, name))
    print(f"  {name:42s} watertight={tm.is_watertight}  size={np.round(tm.extents, 1).tolist()} mm  "
          f"vol={tm.volume/1000:.1f} cm3")
    return tm

# ================= vị trí gripper =================
X_PLATE_IN = FIST_FRONT + FRONT_GAP
X_BASE = X_PLATE_IN + FRONT_T          # mặt lắp horn của follower nằm ở đây
# ma trận follower -> link: z_f -> +X, x_f -> +Z (hàm di động lên trên), y_f -> -Y
R = np.array([[0, 0, 1],
              [0, -1, 0],
              [1, 0, 0]], float)
t = np.array([X_BASE, ARM_AXIS_YZ[0] + TIP_F[1], ARM_AXIS_YZ[1] + TIP_SHIFT_Z - TIP_F[0]])
T_F2L = np.eye(4); T_F2L[:3, :3] = R; T_F2L[:3, 3] = t
def f2l(p): return R @ np.asarray(p, float) + t
TIP_LINK = f2l(TIP_F)

# ================= KHUNG CHỮ U (tấm trên + tay cầm + tấm trước) =================
hx, hy = HOLE_XY
z_top1 = FIST_TOP + PLATE_T
footprint = (m3d.CrossSection.circle(COLLAR_R, SEG).translate((hx, hy))
             + m3d.CrossSection.square((X_BASE - 1.0 - (X_PLATE_IN - 4), 2 * FRONT_HALF_W))
             .translate((X_PLATE_IN - 4, -FRONT_HALF_W))).hull()
top_plate = m3d.Manifold.extrude(footprint, PLATE_T).translate((0, 0, FIST_TOP))
front_plate = box(X_PLATE_IN, X_BASE, -FRONT_HALF_W, FRONT_HALF_W, FIST_BOT, z_top1)

z_tip = FIST_BOT + CLAMP_GAP
handle = (cyl(FIST_TOP - z_tip - TIP_CHAMFER + 0.01, HANDLE_D, z_tip + TIP_CHAMFER)
          + cyl(TIP_CHAMFER, HANDLE_D - 2 * TIP_CHAMFER, z_tip, d2=HANDLE_D)).translate((hx, hy, 0))

bracket = top_plate + front_plate + handle

# lỗ bu lông M5 + rãnh đai ốc ngang trong tay cầm (rãnh mở về phía -X, nằm trong lỗ khi lắp)
bracket -= cyl(BOLT_HOLE_DEPTH, M5_CLEAR, z_tip - 0.1, seg=48).translate((hx, hy, 0))
z_nut = z_tip + NUT_SLOT_FROM_END
bracket -= (hex_prism(M5_NUT_AF, M5_NUT_T, z_nut).rotate((0, 0, 30))
            + box(-HANDLE_D, 0, -M5_NUT_AF / 2, M5_NUT_AF / 2, z_nut, z_nut + M5_NUT_T)).translate((hx, hy, 0))

# 4 lỗ M3 bắt follower (khoan theo -X từ mặt trước tấm trước)
for sx in (-1, 1):
    for sy in (-1, 1):
        c = f2l((HORN_CENTER_F[0] + sx * HORN_HOLE_OFF, HORN_CENTER_F[1] + sy * HORN_HOLE_OFF, 0))
        bracket -= cyl(M3_PILOT_DEPTH + 0.1, M3_PILOT_D, 0, seg=32).rotate((0, -90, 0)) \
            .translate((c[0] + 0.05, c[1], c[2]))

# 2 lỗ M4 ở chân tấm trước (khoan lên +Z)
x_m4 = (X_PLATE_IN + X_BASE) / 2
for sy in (-1, 1):
    bracket -= cyl(M4_PILOT_DEPTH, M4_PILOT_D, FIST_BOT - 0.1, seg=32).translate((x_m4, sy * M4_Y, 0))

# ================= TẤM DƯỚI (rời) =================
zb = FIST_BOT - PLATE_T
bottom = m3d.Manifold.extrude(footprint, PLATE_T).translate((0, 0, zb))
bottom -= cyl(PLATE_T + 0.2, M5_CLEAR, zb - 0.1, seg=48).translate((hx, hy, 0))
bottom -= cyl(M5_HEAD_T + 0.1, M5_HEAD_D, zb - 0.1, seg=48).translate((hx, hy, 0))
for sy in (-1, 1):
    bottom -= cyl(PLATE_T + 0.2, M4_CLEAR_D, zb - 0.1, seg=32).translate((x_m4, sy * M4_Y, 0))
    bottom -= cyl(M4_HEAD_T + 0.1, M4_HEAD_D, zb - 0.1, seg=32).translate((x_m4, sy * M4_Y, 0))

# ================= TEST PLUG (in trước để chọn HANDLE_D) =================
plug = cyl(3, 36, 0)
z = 3
for d in (29.6, 29.2, 28.8):
    plug += cyl(8, d, z); z += 8

# ================= FOLLOWER SO101 đặt vào vị trí =================
fol_tm = trimesh.load(os.path.join(HERE, "SO101", "wrist_roll_follower_so101_v1.stl"))
fol_tm.apply_scale(1000.0)
fol_tm.apply_transform(T_F2L)
follower = from_tm(fol_tm)

# ================= XUẤT =================
for f in os.listdir(OUT):
    if f.endswith(".stl"):
        os.remove(os.path.join(OUT, f))
print("Xuất file:")
bt = save(bracket, "r1_u_bracket.stl")
save(bottom, "r1_u_bottom_plate.stl")
save(bracket + follower, "r1_u_bracket_with_so101_jaw_onepiece.stl")
save(plug, "r1_fit_test_plug.stl")

hand = trimesh.load(os.path.join(HERE, "right_wrist_roll_link.STL")); hand.apply_scale(1000.0)
trimesh.util.concatenate([hand, bt, to_tm(bottom), fol_tm]).export(os.path.join(OUT, "assembly_preview.stl"))
print("  assembly_preview.stl (chỉ để xem, không in)")

# ================= KIỂM TRA =================
print(f"\nMũi hàm cố định (link): X={TIP_LINK[0]:.1f}  Y={TIP_LINK[1]:.2f}  Z={TIP_LINK[2]:.2f}   (trục tay: Y=0, Z=0)")
print(f"Mặt lắp gripper X={X_BASE:.1f}; mũi kẹp cách mặt trước nắm tay {TIP_LINK[0] - FIST_FRONT:.0f} mm")
hv = hand.vertices
fb = fol_tm.bounds
inside = ((hv[:, 0] > fb[0][0]) & (hv[:, 1] > fb[0][1]) & (hv[:, 1] < fb[1][1])
          & (hv[:, 2] > fb[0][2]) & (hv[:, 2] < fb[1][2]))
print(f"Đỉnh tay R1 trong hộp bao gripper: {inside.sum()}  (0 = không va chạm)")
r = np.hypot(hv[:, 0] - hx, hv[:, 1] - hy)
in_fp = (hv[:, 0] > bt.bounds[0][0]) & (hv[:, 0] < X_BASE) & (np.abs(hv[:, 1] - 5) < 40) & (r > HANDLE_D / 2 + 0.5)
print(f"Đỉnh tay R1 lấn vào tấm trên/dưới: {(in_fp & ((hv[:, 2] > FIST_TOP + 0.05) | (hv[:, 2] < FIST_BOT - 0.05))).sum()}")
print(f"Đỉnh tay R1 lấn vào tấm trước: {((hv[:, 0] > X_PLATE_IN) & (np.abs(hv[:, 1]) < FRONT_HALF_W)).sum()}")
