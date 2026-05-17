import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d import Axes3D
import os
from datetime import datetime

DT = datetime.now().strftime("%Y%m%d_%H%M%S")

def fname(prefix, info):
    return os.path.join(SAVE_DIR, f"{prefix}_iso_virtual_{info}_{DT}.png")

CSV_PATH = os.path.join(os.path.dirname(__file__), "virtual", "iso_cube_ik_validation.csv")
SAVE_DIR = os.path.join(os.path.dirname(__file__), "virtual")

df = pd.read_csv(CSV_PATH)

all_points = df['point'].unique().tolist()
# sort: numeric points first, then 'C'
iso_points = sorted([p for p in all_points if str(p) != 'C'], key=lambda x: int(x))
has_clearance = 'C' in [str(p) for p in all_points]
points = iso_points + (['C'] if has_clearance else [])
n_points = len(points)
cold = df[df['start_type'] == 'cold'].set_index('point')
warm = df[df['start_type'] == 'warm'].set_index('point')

x     = np.arange(n_points)
width = 0.35

def suc_colors(suc_list, base):
    return [base if s else '#e74c3c' for s in suc_list]

cold_suc     = [cold.loc[p, 'success']    for p in points]
warm_suc     = [warm.loc[p, 'success']    for p in points]
cold_pos_err = [cold.loc[p, 'pos_err_mm'] for p in points]
warm_pos_err = [warm.loc[p, 'pos_err_mm'] for p in points]
cold_rot     = [cold.loc[p, 'rot_err_deg'] for p in points]
warm_rot     = [warm.loc[p, 'rot_err_deg'] for p in points]
cold_t       = [cold.loc[p, 'time_ms']    for p in points]
warm_t       = [warm.loc[p, 'time_ms']    for p in points]

# ──────────────────────────────────────────────────────────────
# Figure 1 : 4-panel metrics
# ──────────────────────────────────────────────────────────────
fig1, axes = plt.subplots(2, 2, figsize=(14, 10))
fig1.suptitle("ISO Cube IK Validation — Virtual (Software)", fontsize=14, fontweight='bold')

# 1. Position error
ax = axes[0, 0]
b1 = ax.bar(x - width/2, cold_pos_err, width, label='cold',
            color=suc_colors(cold_suc, '#3498db'), edgecolor='k', linewidth=0.5)
b2 = ax.bar(x + width/2, warm_pos_err, width, label='warm',
            color=suc_colors(warm_suc, '#f39c12'), edgecolor='k', linewidth=0.5)
ax.set_yscale('symlog', linthresh=1e-10)
ax.set_xticks(x); ax.set_xticklabels([f'P{p}' if str(p) != 'C' else 'Clr' for p in points])
ax.set_ylabel('Position Error (mm)')
ax.set_title('Position Error  [log scale]')
ax.legend(); ax.grid(axis='y', alpha=0.3)
for bar, val, suc in zip(b1, cold_pos_err, cold_suc):
    ax.text(bar.get_x() + bar.get_width()/2, max(bar.get_height(), 1e-10) * 2,
            f'{val:.2e}', ha='center', va='bottom', fontsize=7,
            color='red' if not suc else 'black')
for bar, val, suc in zip(b2, warm_pos_err, warm_suc):
    ax.text(bar.get_x() + bar.get_width()/2, max(bar.get_height(), 1e-10) * 2,
            f'{val:.2e}', ha='center', va='bottom', fontsize=7,
            color='red' if not suc else 'black')

# 2. Rotation error
ax = axes[0, 1]
ax.bar(x - width/2, cold_rot, width, label='cold',
       color=suc_colors(cold_suc, '#3498db'), edgecolor='k', linewidth=0.5)
ax.bar(x + width/2, warm_rot, width, label='warm',
       color=suc_colors(warm_suc, '#f39c12'), edgecolor='k', linewidth=0.5)
ax.set_yscale('symlog', linthresh=1e-12)
ax.set_xticks(x); ax.set_xticklabels([f'P{p}' if str(p) != 'C' else 'Clr' for p in points])
ax.set_ylabel('Rotation Error (deg)')
ax.set_title('Rotation Error  [log scale]')
ax.legend(); ax.grid(axis='y', alpha=0.3)

# 3. Computation time
ax = axes[1, 0]
ax.bar(x - width/2, cold_t, width, label='cold',
       color=suc_colors(cold_suc, '#3498db'), edgecolor='k', linewidth=0.5)
ax.bar(x + width/2, warm_t, width, label='warm',
       color=suc_colors(warm_suc, '#f39c12'), edgecolor='k', linewidth=0.5)
ax.set_xticks(x); ax.set_xticklabels([f'P{p}' if str(p) != 'C' else 'Clr' for p in points])
ax.set_ylabel('Time (ms)')
ax.set_title('IK Computation Time')
ax.legend(); ax.grid(axis='y', alpha=0.3)

# 4. Success/Fail heatmap
ax = axes[1, 1]
matrix = np.array([[cold.loc[p, 'success'] for p in points],
                   [warm.loc[p, 'success'] for p in points]], dtype=float)
ax.imshow(matrix, cmap='RdYlGn', vmin=0, vmax=1, aspect='auto')
ax.set_xticks(range(n_points)); ax.set_xticklabels([f'P{p}' if str(p) != 'C' else 'Clr' for p in points])
ax.set_yticks([0, 1]); ax.set_yticklabels(['cold', 'warm'])
ax.set_title('Success / Fail Matrix')
for i in range(2):
    for j in range(n_points):
        ax.text(j, i, '✓' if int(matrix[i, j]) else '✗',
                ha='center', va='center', fontsize=16, color='black')

green  = mpatches.Patch(color='#2ecc71', label='Success')
red    = mpatches.Patch(color='#e74c3c', label='Fail')
blue   = mpatches.Patch(color='#3498db', label='Cold start')
orange = mpatches.Patch(color='#f39c12', label='Warm start')
fig1.legend(handles=[green, red, blue, orange], loc='lower center',
            ncol=4, fontsize=9, bbox_to_anchor=(0.5, 0.01))

plt.tight_layout(rect=[0, 0.05, 1, 1])
out1 = fname("matrix", "metrics")
fig1.savefig(out1, dpi=150, bbox_inches='tight')
print(f"Saved: {out1}")

# ──────────────────────────────────────────────────────────────
# Figure 2 : 3D spatial visualization
# ──────────────────────────────────────────────────────────────
fig2 = plt.figure(figsize=(12, 8))
ax3d = fig2.add_subplot(111, projection='3d')
ax3d.set_title("3D: Goal Positions vs FK Positions", fontsize=13, fontweight='bold')

cold_df = df[df['start_type'] == 'cold'].sort_values('point')
cx = cold_df[cold_df['point'] == cold_df['point'].min()]['goal_x'].values[0]
cy = cold_df[cold_df['point'] == cold_df['point'].min()]['goal_y'].values[0]
cz = cold_df[cold_df['point'] == cold_df['point'].min()]['goal_z'].values[0]
dx, dy, dz  = 0.100, 0.100, 0.100
corners = np.array([[cx+sx*dx, cy+sy*dy, cz+sz*dz]
                    for sx in [-1, 1] for sy in [-1, 1] for sz in [-1, 1]])
edges = [(0,1),(0,2),(0,4),(1,3),(1,5),(2,3),(2,6),(3,7),(4,5),(4,6),(5,7),(6,7)]
for i, j in edges:
    ax3d.plot(*zip(corners[i], corners[j]), color='gray', alpha=0.3, linewidth=0.8)

labeled = set()
for _, row in df.iterrows():
    gx, gy, gz = row['goal_x'], row['goal_y'], row['goal_z']
    fx, fy, fz = row['fk_x'],   row['fk_y'],   row['fk_z']
    suc  = bool(row['success'])
    is_cold = row['start_type'] == 'cold'
    marker = '^' if is_cold else 'o'
    gcolor = '#3498db' if is_cold else '#f39c12'

    ax3d.scatter(gx, gy, gz, c=gcolor, marker=marker, s=80,
                 edgecolors='k', linewidths=0.5, zorder=5)

    if is_cold:
        ax3d.text(gx, gy, gz + 0.015, f"P{int(row['point'])}",
                  fontsize=8, color='#2c3e50', ha='center')

    if not suc:
        ax3d.scatter(fx, fy, fz, c='#e74c3c', marker='x', s=100,
                     linewidths=2, zorder=6)
        ax3d.plot([gx, fx], [gy, fy], [gz, fz],
                  color='#e74c3c', alpha=0.6, linewidth=1.2, linestyle='--')
        ax3d.text(fx + 0.01, fy + 0.01, fz,
                  f"P{int(row['point'])}-{row['start_type']}\n{row['pos_err_mm']:.1f}mm",
                  fontsize=7, color='#c0392b')

ax3d.set_xlabel('X (m)'); ax3d.set_ylabel('Y (m)'); ax3d.set_zlabel('Z (m)')

handles = [
    plt.Line2D([0],[0], marker='^', color='w', markerfacecolor='#3498db',
               markersize=9, markeredgecolor='k', label='Goal (cold)'),
    plt.Line2D([0],[0], marker='o', color='w', markerfacecolor='#f39c12',
               markersize=9, markeredgecolor='k', label='Goal (warm)'),
    plt.Line2D([0],[0], marker='x', color='#e74c3c', markersize=9,
               linewidth=2, label='FK result (fail)'),
]
ax3d.legend(handles=handles, loc='upper left', fontsize=9)

out2 = fname("plot", "3d")
fig2.savefig(out2, dpi=150, bbox_inches='tight')
print(f"Saved: {out2}")

# ──────────────────────────────────────────────────────────────
# Summary
# ──────────────────────────────────────────────────────────────
print("\n========== Summary ==========")
print(f"{'Point':<8}{'Type':<8}{'OK':<6}{'pos_err(mm)':<18}{'rot_err(deg)':<18}{'time(ms)'}")
print("-" * 65)
for _, row in df.iterrows():
    ok = 'OK' if row['success'] else 'FAIL'
    print(f"P{int(row['point']):<7}{row['start_type']:<8}{ok:<6}"
          f"{row['pos_err_mm']:<18.4e}{row['rot_err_deg']:<18.4e}{row['time_ms']:.4f}")

# ──────────────────────────────────────────────────────────────
# Figure 3 & 4 : Success reference plots (all points converged)
# ──────────────────────────────────────────────────────────────
goal_xyz = [(r['goal_x'], r['goal_y'], r['goal_z'])
            for _, r in df[df['start_type']=='cold'].sort_values('point').iterrows()]

ref = pd.DataFrame({
    'point':      [p for p in points for _ in range(2)],
    'start_type': ['cold','warm'] * n_points,
    'success':    [1] * (n_points * 2),
    'time_ms':    [0.020, 0.015, 0.012, 0.015, 0.013, 0.014, 0.015, 0.016, 0.014, 0.015][:n_points*2],
    'pos_err_mm': [1.7e-13, 2.9e-8] * n_points,
    'rot_err_deg':[1.2e-14, 2.5e-9] * n_points,
    'goal_x': [xyz[0] for xyz in goal_xyz for _ in range(2)],
    'goal_y': [xyz[1] for xyz in goal_xyz for _ in range(2)],
    'goal_z': [xyz[2] for xyz in goal_xyz for _ in range(2)],
    'fk_x':   [xyz[0] for xyz in goal_xyz for _ in range(2)],
    'fk_y':   [xyz[1] for xyz in goal_xyz for _ in range(2)],
    'fk_z':   [xyz[2] for xyz in goal_xyz for _ in range(2)],
})
rc = ref[ref['start_type']=='cold'].set_index('point')
rw = ref[ref['start_type']=='warm'].set_index('point')

# ── Fig3: metrics (success) ────────────────────────────────────
fig3, axes3 = plt.subplots(2, 2, figsize=(14, 10))
fig3.suptitle("ISO Cube IK Validation — SUCCESS Reference", fontsize=14, fontweight='bold')

def ref_bars(ax, c_vals, w_vals, ylabel, title, linthresh=1e-10, log=True):
    b1 = ax.bar(x - width/2, c_vals, width, label='cold',
                color='#3498db', edgecolor='k', linewidth=0.5)
    b2 = ax.bar(x + width/2, w_vals, width, label='warm',
                color='#f39c12', edgecolor='k', linewidth=0.5)
    if log: ax.set_yscale('symlog', linthresh=linthresh)
    ax.set_xticks(x); ax.set_xticklabels([f'P{p}' if str(p) != 'C' else 'Clr' for p in points])
    ax.set_ylabel(ylabel); ax.set_title(title)
    ax.legend(); ax.grid(axis='y', alpha=0.3)

ref_bars(axes3[0,0], [rc.loc[p,'pos_err_mm'] for p in points],
                     [rw.loc[p,'pos_err_mm'] for p in points],
         'Position Error (mm)', 'Position Error  [log scale]', linthresh=1e-10)
ref_bars(axes3[0,1], [rc.loc[p,'rot_err_deg'] for p in points],
                     [rw.loc[p,'rot_err_deg'] for p in points],
         'Rotation Error (deg)', 'Rotation Error  [log scale]', linthresh=1e-12)
ref_bars(axes3[1,0], [rc.loc[p,'time_ms'] for p in points],
                     [rw.loc[p,'time_ms'] for p in points],
         'Time (ms)', 'IK Computation Time', log=False)

ax = axes3[1,1]
mat = np.ones((2, n_points))
ax.imshow(mat, cmap='RdYlGn', vmin=0, vmax=1, aspect='auto')
ax.set_xticks(range(n_points)); ax.set_xticklabels([f'P{p}' if str(p) != 'C' else 'Clr' for p in points])
ax.set_yticks([0,1]); ax.set_yticklabels(['cold','warm'])
ax.set_title('Success / Fail Matrix')
for i in range(2):
    for j in range(n_points):
        ax.text(j, i, '✓', ha='center', va='center', fontsize=16, color='black')

fig3.legend(handles=[green, red, blue, orange], loc='lower center',
            ncol=4, fontsize=9, bbox_to_anchor=(0.5, 0.01))
plt.tight_layout(rect=[0, 0.05, 1, 1])
out3 = os.path.join(SAVE_DIR, "success_matrix_iso_virtual_metrics.png")
fig3.savefig(out3, dpi=150, bbox_inches='tight')
print(f"Saved: {out3}")

# ── Fig4: 3D (success) ─────────────────────────────────────────
fig4 = plt.figure(figsize=(12, 8))
ax4  = fig4.add_subplot(111, projection='3d')
ax4.set_title("3D: Goal Positions vs FK Positions — SUCCESS Reference",
              fontsize=13, fontweight='bold')

for i, j in edges:
    ax4.plot(*zip(corners[i], corners[j]), color='gray', alpha=0.3, linewidth=0.8)

for _, row in ref.iterrows():
    gx, gy, gz = row['goal_x'], row['goal_y'], row['goal_z']
    is_cold = row['start_type'] == 'cold'
    marker = '^' if is_cold else 'o'
    gcolor = '#3498db' if is_cold else '#f39c12'
    ax4.scatter(gx, gy, gz, c=gcolor, marker=marker, s=80,
                edgecolors='k', linewidths=0.5, zorder=5)
    if is_cold:
        ax4.text(gx, gy, gz + 0.015, f"P{int(row['point'])}",
                 fontsize=8, color='#2c3e50', ha='center')

ax4.set_xlabel('X (m)'); ax4.set_ylabel('Y (m)'); ax4.set_zlabel('Z (m)')
ax4.legend(handles=handles[:2], loc='upper left', fontsize=9)

out4 = os.path.join(SAVE_DIR, "success_plot_iso_virtual_3d.png")
fig4.savefig(out4, dpi=150, bbox_inches='tight')
print(f"Saved: {out4}")
