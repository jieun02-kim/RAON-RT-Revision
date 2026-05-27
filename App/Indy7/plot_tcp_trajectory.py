import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import glob
import os

LOG_DIR  = "rt_log_results"
PLOT_DIR = os.path.join(LOG_DIR, "plots")
os.makedirs(PLOT_DIR, exist_ok=True)

files = sorted(glob.glob(os.path.join(LOG_DIR, "DataLog_*.csv")))
if not files:
    print("ERROR: rt_log_results/DataLog_*.csv 파일이 없습니다.")
    exit(1)

csv_file = files[-1]
print(f"Loading: {csv_file}")
df = pd.read_csv(csv_file)
df['t'] = (df['timestamp_ns'] - df['timestamp_ns'].iloc[0]) * 1e-9

if df['tcp_x'].abs().max() < 1e-6 and df['tcp_y'].abs().max() < 1e-6 and df['tcp_z'].abs().max() < 1e-6:
    print("WARNING: tcp_x/y/z가 모두 0입니다. IK 모드(m/i/a)에서 l키를 눌러 기록하셨나요?")

fig = plt.figure(figsize=(14, 5))

# 3D 궤적
ax1 = fig.add_subplot(131, projection='3d')
ax1.plot(df['tcp_x'], df['tcp_y'], df['tcp_z'], lw=1.0)
ax1.scatter(df['tcp_x'].iloc[0],  df['tcp_y'].iloc[0],  df['tcp_z'].iloc[0],  c='g', s=40, label='start')
ax1.scatter(df['tcp_x'].iloc[-1], df['tcp_y'].iloc[-1], df['tcp_z'].iloc[-1], c='r', s=40, label='end')
ax1.set_xlabel('X (m)'); ax1.set_ylabel('Y (m)'); ax1.set_zlabel('Z (m)')
ax1.set_title('TCP 3D Trajectory')
ax1.legend()

# X/Y/Z vs time
ax2 = fig.add_subplot(132)
ax2.plot(df['t'], df['tcp_x'], label='X')
ax2.plot(df['t'], df['tcp_y'], label='Y')
ax2.plot(df['t'], df['tcp_z'], label='Z')
ax2.set_xlabel('Time (s)'); ax2.set_ylabel('Position (m)')
ax2.set_title('TCP Position vs Time')
ax2.legend()

# XY 평면 투영
ax3 = fig.add_subplot(133)
ax3.plot(df['tcp_x'], df['tcp_y'])
ax3.scatter(df['tcp_x'].iloc[0],  df['tcp_y'].iloc[0],  c='g', s=40, label='start')
ax3.scatter(df['tcp_x'].iloc[-1], df['tcp_y'].iloc[-1], c='r', s=40, label='end')
ax3.set_xlabel('X (m)'); ax3.set_ylabel('Y (m)')
ax3.set_title('TCP XY Projection')
ax3.legend(); ax3.set_aspect('equal')

plt.tight_layout()
png_name = os.path.join(PLOT_DIR, os.path.splitext(os.path.basename(csv_file))[0] + "_tcp.png")
plt.savefig(png_name, dpi=150)
print(f"Plot saved: {png_name}")
plt.show()
