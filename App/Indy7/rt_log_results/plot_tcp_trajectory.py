import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from mpl_toolkits.mplot3d import Axes3D
import glob
import os
import shutil
import numpy as np
from datetime import datetime

PLOT_DIR = "plots"
DONE_DIR = "plotted_csv"
TAIL     = 500   # 마지막 500행(0.5s)을 "정착 위치"로 사용
os.makedirs(PLOT_DIR, exist_ok=True)
os.makedirs(DONE_DIR, exist_ok=True)

# ── 데이터 수집 ───────────────────────────────────────────────
files = sorted(glob.glob("DataLog_*.csv"))
if not files:
    print("ERROR: DataLog_*.csv 파일이 없습니다.")
    exit(1)

print(f"총 {len(files)}개 파일 로드")
colors   = cm.tab10(np.linspace(0, 1, max(len(files), 2)))
datasets = []
for f in files:
    df    = pd.read_csv(f)
    df['t'] = (df['timestamp_ns'] - df['timestamp_ns'].iloc[0]) * 1e-9
    label = os.path.splitext(os.path.basename(f))[0].replace("DataLog_", "")
    datasets.append((label, df))
    print(f"  loaded: {f}  ({len(df)} rows)")

# ── RPY 유틸 ──────────────────────────────────────────────────
def rpy_to_rot(r, p, y):
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    return np.array([
        [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [-sp,    cp*sr,             cp*cr            ]
    ])

def rot_error_deg(rpy_goal, rpy_actual):
    Rg = rpy_to_rot(*rpy_goal)
    Ra = rpy_to_rot(*rpy_actual)
    R_err = Rg @ Ra.T
    val   = np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0)
    return np.degrees(np.arccos(val))

GOAL_COLS   = ['goal_x',    'goal_y',    'goal_z',
               'goal_roll', 'goal_pitch','goal_yaw']
ORIENT_COLS = ['tcp_roll',  'tcp_pitch', 'tcp_yaw']
HAS_GOAL    = all(c in datasets[0][1].columns for c in GOAL_COLS) if datasets else False

# ── Figure 1 : 궤적 시각화 ───────────────────────────────────
fig1 = plt.figure(figsize=(16, 5))
ax1  = fig1.add_subplot(131, projection='3d')
ax2  = fig1.add_subplot(132)
ax3  = fig1.add_subplot(133)

for (label, df), color in zip(datasets, colors):
    x, y, z, t = df['tcp_x'], df['tcp_y'], df['tcp_z'], df['t']

    if x.abs().max() < 1e-6 and y.abs().max() < 1e-6 and z.abs().max() < 1e-6:
        print(f"  WARNING [{label}]: tcp 모두 0 — IK 모드에서 l키로 기록했나요?")

    ax1.plot(x, y, z, lw=1.2, color=color, label=label)
    ax1.scatter(x.iloc[0],  y.iloc[0],  z.iloc[0],  c=[color], s=30, marker='o')
    ax1.scatter(x.iloc[-1], y.iloc[-1], z.iloc[-1], c=[color], s=30, marker='x')

    ax2.plot(t, x, lw=1.0, color=color, linestyle='-')
    ax2.plot(t, y, lw=1.0, color=color, linestyle='--')
    ax2.plot(t, z, lw=1.0, color=color, linestyle=':')

    ax3.plot(x, y, lw=1.2, color=color, label=label)
    ax3.scatter(x.iloc[0],  y.iloc[0],  c=[color], s=30, marker='o')
    ax3.scatter(x.iloc[-1], y.iloc[-1], c=[color], s=30, marker='x')

ax1.set_xlabel('X (m)'); ax1.set_ylabel('Y (m)'); ax1.set_zlabel('Z (m)')
ax1.set_title('TCP 3D Trajectory (all runs)')
ax1.legend(fontsize=6)

ax2.set_xlabel('Time (s)'); ax2.set_ylabel('Position (m)')
ax2.set_title('TCP Position vs Time')
style_h = [plt.Line2D([0],[0], color='k', ls='-',  label='X'),
           plt.Line2D([0],[0], color='k', ls='--', label='Y'),
           plt.Line2D([0],[0], color='k', ls=':',  label='Z')]
file_h  = [plt.Line2D([0],[0], color=c, lw=1.5, label=lbl)
           for (lbl,_), c in zip(datasets, colors)]
ax2.legend(handles=file_h + style_h, fontsize=6)

ax3.set_xlabel('X (m)'); ax3.set_ylabel('Y (m)')
ax3.set_title('TCP XY Projection')
ax3.legend(fontsize=6); ax3.set_aspect('equal')

fig1.tight_layout()

# ── Figure 2 : 회차별 오차 분석 ──────────────────────────────
fig2, (ax_pos, ax_rot) = plt.subplots(1, 2, figsize=(12, 4))
fig2.suptitle('Per-Run Error vs Goal  (last 500 rows = settled state, ~0.5 s)', fontsize=11)

labels_err = []
pos_errors = []   # mm  (absolute vs goal, or fallback deviation)
rot_errors = []   # deg (absolute vs goal)

# 전체 runs의 평균 최종위치 (fallback용)
final_positions = np.array([
    df.tail(TAIL)[['tcp_x','tcp_y','tcp_z']].mean().values
    for _, df in datasets
])
mean_final_pos = final_positions.mean(axis=0)

for (label, df), fp in zip(datasets, final_positions):
    tail = df.tail(TAIL)
    labels_err.append(label)

    has_goal_this = HAS_GOAL and all(c in df.columns for c in GOAL_COLS + ORIENT_COLS)
    if has_goal_this:
        goal_pos = tail[GOAL_COLS[:3]].mean().values
        goal_valid = np.linalg.norm(goal_pos) > 1e-4
    else:
        goal_valid = False

    if has_goal_this and goal_valid:
        goal_rpy  = tail[GOAL_COLS[3:]].mean().values
        final_rpy = tail[ORIENT_COLS].mean().values
        pos_err_mm  = np.linalg.norm(fp - goal_pos) * 1000.0
        rot_err_deg = rot_error_deg(goal_rpy, final_rpy)
        pos_errors.append(pos_err_mm)
        rot_errors.append(rot_err_deg)
        print(f"  [{label}]  pos_err={pos_err_mm:.3f} mm  rot_err={rot_err_deg:.4f} deg")
    else:
        # fallback: 평균 최종위치 대비 편차 (반복성)
        dev_mm = np.linalg.norm(fp - mean_final_pos) * 1000.0
        pos_errors.append(dev_mm)
        rot_errors.append(np.nan)
        reason = "goal at origin" if has_goal_this else "no goal columns"
        print(f"  [{label}]  fallback pos_dev={dev_mm:.3f} mm  ({reason})")

# pos 차트 모드 결정
use_absolute = any(not np.isnan(v) for v in rot_errors)  # rot_errors 유효 = goal 있음
pos_title = ('Position Norm Error  ||TCP_final - Goal|| (mm)'
             if use_absolute else
             'Position Deviation from Mean Final Pos (mm)  [no valid goal -- fallback to repeatability]')

x_idx = np.arange(len(labels_err))
bar_w = 0.5

# ── Position error bar ────────────────────────────────────────
bars = ax_pos.bar(x_idx, pos_errors, width=bar_w,
                  color=[colors[i] for i in range(len(datasets))],
                  edgecolor='black', linewidth=0.5)
ax_pos.set_xticks(x_idx)
ax_pos.set_xticklabels(labels_err, rotation=25, ha='right', fontsize=7)
ax_pos.set_ylabel('Position Error (mm)')
ax_pos.set_title(pos_title)
ax_pos.yaxis.grid(True, linestyle='--', alpha=0.5)
ax_pos.set_axisbelow(True)
for bar, val in zip(bars, pos_errors):
    if not np.isnan(val):
        ax_pos.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.01,
                    f'{val:.3f}', ha='center', va='bottom', fontsize=7)

# ── Orientation error bar ─────────────────────────────────────
if use_absolute:
    bars2 = ax_rot.bar(x_idx, rot_errors, width=bar_w,
                       color=[colors[i] for i in range(len(datasets))],
                       edgecolor='black', linewidth=0.5)
    ax_rot.set_xticks(x_idx)
    ax_rot.set_xticklabels(labels_err, rotation=25, ha='right', fontsize=7)
    ax_rot.set_ylabel('Orientation Error (deg)')
    ax_rot.set_title('Orientation Norm Error  angle(R_final, R_goal) (deg)')
    ax_rot.yaxis.grid(True, linestyle='--', alpha=0.5)
    ax_rot.set_axisbelow(True)
    for bar, val in zip(bars2, rot_errors):
        if not np.isnan(val):
            ax_rot.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.001,
                        f'{val:.4f}', ha='center', va='bottom', fontsize=7)
else:
    msg = ('No valid goal pose in CSV.\n'
           'Rebuild C++ and record after setting IK target\n'
           'to get orientation error.')
    ax_rot.text(0.5, 0.5, msg,
                ha='center', va='center', transform=ax_rot.transAxes,
                fontsize=9, color='gray')
    ax_rot.set_title('Orientation Norm Error (deg)')
    ax_rot.set_xticks([]); ax_rot.set_yticks([])

fig2.tight_layout()

# ── 저장 ─────────────────────────────────────────────────────
stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
out1  = os.path.join(PLOT_DIR, f"combined_{stamp}.png")
out2  = os.path.join(PLOT_DIR, f"error_{stamp}.png")
fig1.savefig(out1, dpi=150)
fig2.savefig(out2, dpi=150)
print(f"Plot saved: {out1}")
print(f"Plot saved: {out2}")

# 현재 디렉터리의 새 CSV만 이동
for f in sorted(glob.glob("DataLog_*.csv")):
    shutil.move(f, os.path.join(DONE_DIR, os.path.basename(f)))
    print(f"CSV moved: {DONE_DIR}/{os.path.basename(f)}")

plt.show()
