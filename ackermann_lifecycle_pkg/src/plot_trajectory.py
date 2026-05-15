"""
Plot comparison between DiffDrive and Ackermann from trajectory.csv
Run: python3 plot_trajectory.py
"""

import csv
import math
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# ── Read CSV ──────────────────────────────────
data = {k: [] for k in [
    "time",
    "x_diff", "y_diff", "theta_diff",
    "x_ackermann", "y_ackermann", "theta_ackermann"
]}

with open("/home/ngxwang/ros2_ws/src/ackermann_lifecycle_pkg/trajectory.csv") as f:
    reader = csv.DictReader(f)
    for row in reader:
        for k in data:
            data[k].append(float(row[k]))

t        = data["time"]
x_d      = data["x_diff"];        y_d  = data["y_diff"]
x_a      = data["x_ackermann"];   y_a  = data["y_ackermann"]
th_d     = data["theta_diff"];    th_a = data["theta_ackermann"]

# ── Plot ──────────────────────────────────────
fig = plt.figure(figsize=(15, 5))
fig.suptitle("Ackermann vs Differential Drive — Kinematics Comparison", fontsize=14, fontweight="bold")
gs = gridspec.GridSpec(1, 3, figure=fig, wspace=0.3)

# 1. Trajectory XY
ax1 = fig.add_subplot(gs[0, 0])
ax1.plot(x_d, y_d, "b--", linewidth=1.8, label="Differential Drive")
ax1.plot(x_a, y_a, "r-",  linewidth=2.0, label="Ackermann")
ax1.plot(x_d[0], y_d[0], "go", markersize=8, label="Start")
ax1.plot(x_d[-1], y_d[-1], "bs", markersize=8)
ax1.plot(x_a[-1], y_a[-1], "rs", markersize=8, label="End")
ax1.set_xlabel("x (m)"); ax1.set_ylabel("y (m)")
ax1.set_title("Trajectory (XY)")
ax1.legend(); ax1.grid(True); ax1.set_aspect("equal")

# 2. Position Difference
ax2 = fig.add_subplot(gs[0, 1])
dx = [x_a[i] - x_d[i] for i in range(len(t))]
dy = [y_a[i] - y_d[i] for i in range(len(t))]
dist = [math.sqrt(dx[i]**2 + dy[i]**2) for i in range(len(t))]
ax2.plot(t, dist, "purple", linewidth=1.8)
ax2.set_xlabel("Time (s)"); ax2.set_ylabel("Δ distance (m)")
ax2.set_title("Position Difference\n(Ackermann − DiffDrive)")
ax2.grid(True)

# 3. Heading Angle (Theta) over time
ax3 = fig.add_subplot(gs[0, 2])
ax3.plot(t, [math.degrees(v) for v in th_d], "b--", label="DiffDrive")
ax3.plot(t, [math.degrees(v) for v in th_a], "r-",  label="Ackermann")
ax3.set_xlabel("Time (s)"); ax3.set_ylabel("θ (degrees)")
ax3.set_title("Heading Angle θ")
ax3.legend(); ax3.grid(True)

plt.savefig("/home/ngxwang/ros2_ws/src/ackermann_lifecycle_pkg/trajectory_comparison.png", dpi=150, bbox_inches="tight")
plt.show()
print("[OK] Saved trajectory_comparison.png")