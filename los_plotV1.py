import numpy as np
import matplotlib.pyplot as plt

# ===== PARAM =====
dt = 0.05
V = 3##############
L = 1################
Kp_roll = 1.5################
max_roll = 20#######################

alpha = 0.65   # weight ของ LOS (0=direct only, 1=LOS only) tuneeee
D_path = 1  # ความยาว path ที่สร้างจาก heading

# ===== STATE =====
x, y = -10, 0
yaw = 135  # initial heading

# target
x_target, y_target = 35, 15

trajectory = []
yaw_history = []
yaw_target_history = []
roll_history = []

def wrap(angle):
    return (angle + np.pi) % (2*np.pi) - np.pi

# ===== SIMULATION =====
for i in range(1500):

    # =====================================================
    # 🔥 1. สร้าง "path อัตโนมัติ" จาก heading ปัจจุบัน
    # =====================================================
    x_start = x
    y_start = y

    x_end = x + D_path * np.cos(yaw)
    y_end = y + D_path * np.sin(yaw)

    # path heading (ทิศที่เครื่องกำลังหัน)
    path_heading = np.arctan2(y_end - y_start, x_end - x_start)

    # =====================================================
    # 🔥 2. คำนวณ cross-track error (อิง path ที่สร้าง)
    # =====================================================
    dx = x - x_start
    dy = y - y_start
    e = -np.sin(path_heading)*dx + np.cos(path_heading)*dy

    # =====================================================
    # 🔥 3. LOS guidance (ตาม path)
    # =====================================================
    yaw_los = path_heading - np.arctan2(e, L)
    yaw_los = wrap(yaw_los)

    # =====================================================
    # 🔥 4. Direct-to-target guidance (วิ่งเข้าจุดตรง)
    # =====================================================
    yaw_direct = np.arctan2(y_target - y, x_target - x)

    # =====================================================
    # 🔥 5. Hybrid blending (สำคัญมาก)
    # =====================================================
    yaw_target = alpha * yaw_los + (1 - alpha) * yaw_direct
    yaw_target = wrap(yaw_target)

    # =====================================================
    # 🔥 6. Roll control (inner loop)
    # =====================================================
    yaw_error = wrap(yaw_target - yaw)
    roll = Kp_roll * yaw_error

    # simple aircraft turn model
    yaw_rate = np.tan(roll) * 0.5
    yaw += yaw_rate * dt

    # =====================================================
    # 🔥 7. Motion update
    # =====================================================
    x += V * np.cos(yaw) * dt
    y += V * np.sin(yaw) * dt

    # save
    trajectory.append((x, y))
    yaw_history.append(yaw)
    yaw_target_history.append(yaw_target)
    roll_history.append(roll)

# ===== CONVERT =====
traj = np.array(trajectory)

# ===== FIGURE 1 =====
fig, ax = plt.subplots()

# trajectory
ax.plot(traj[:,0], traj[:,1], label="trajectory", zorder=1)

# target
ax.scatter(x_target, y_target, label="target")

# ===== ARROWS =====
skip = 20
for i in range(0, len(traj), skip):
    px, py = traj[i]
    yaw_i = yaw_history[i]
    roll_i = roll_history[i]

    dx_arrow = np.cos(yaw_i)
    dy_arrow = np.sin(yaw_i)

    scale = 1 + abs(roll_i)*3

    # normalize roll → color
    roll_deg = np.rad2deg(roll_i)   # 🔥 แปลงก่อน
    roll_norm = abs(roll_deg) / max_roll
    roll_norm = np.clip(roll_norm, 0, 1)

    color = plt.cm.RdYlGn_r(roll_norm)

    ax.arrow(px, py,
             dx_arrow*scale,
             dy_arrow*scale,
             head_width=0.5,
             head_length=0.7,
             color=color,
             zorder=3)

# ===== SETTINGS =====
ax.set_aspect('equal')
ax.set_xlabel("X Position (m)")
ax.set_ylabel("Y Position (m)")
ax.set_title("Hybrid LOS (Dynamic Path + Direct Target)")
ax.grid()
ax.legend()

# ===== COLORBAR =====
sm = plt.cm.ScalarMappable(cmap="RdYlGn_r",
                           norm=plt.Normalize(vmin=0, vmax=max_roll))
sm.set_array([])
fig.colorbar(sm, ax=ax, label="Roll magnitude (Degree)")

plt.show()

# # ===== FIGURE 2 =====
# plt.figure()
# plt.plot(yaw_history, label="yaw (actual)")
# plt.plot(yaw_target_history, '--', label="yaw_target (hybrid)")
# plt.title("Yaw vs Target (Hybrid)")
# plt.grid()
# plt.legend()
# plt.show()

# # ===== FIGURE 3 =====
# plt.figure()
# plt.plot(roll_history)
# plt.title("Roll Command")
# plt.grid()
# plt.show()