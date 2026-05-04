import matplotlib
matplotlib.use('TkAgg')
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import sys
import shutil

SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
PLOT_DIR    = os.path.join(SCRIPT_DIR, "plots")
DONE_DIR    = os.path.join(SCRIPT_DIR, "plotted_csv")
CSV_PATH    = os.path.join(SCRIPT_DIR, "ik_accuracy_log.csv")

os.makedirs(PLOT_DIR, exist_ok=True)
os.makedirs(DONE_DIR, exist_ok=True)

if not os.path.exists(CSV_PATH):
    print(f"ERROR: {CSV_PATH} not found. Run IK trials first.")
    sys.exit(1)

df = pd.read_csv(CSV_PATH)
n  = len(df)
print(f"Loaded {n} trials")
print(df[['err_x', 'err_y', 'err_z', 'norm_err', 'err_r', 'err_p', 'err_y', 'rot_norm_err']].describe())

trials   = np.arange(1, n + 1)
mean_err     = df['norm_err'].mean()
std_err      = df['norm_err'].std()
mean_rot_err = df['rot_norm_err'].mean()
std_rot_err  = df['rot_norm_err'].std()

fig, axes = plt.subplots(4, 1, figsize=(10, 14))

# 1. 위치 축별 오차
axes[0].plot(trials, df['err_x'] * 1000, marker='o', label='eX')
axes[0].plot(trials, df['err_y'] * 1000, marker='s', label='eY')
axes[0].plot(trials, df['err_z'] * 1000, marker='^', label='eZ')
axes[0].axhline(0, color='black', linewidth=0.5, linestyle='--')
axes[0].set_ylabel('Error (mm)')
axes[0].set_title('IK Accuracy — Position Per-Axis Error')
axes[0].legend()
axes[0].grid(True)

# 2. 위치 Norm 오차
axes[1].bar(trials, df['norm_err'] * 1000, color='steelblue', label='Pos Norm Error')
axes[1].axhline(mean_err * 1000, color='red', linewidth=1.5, linestyle='--',
                label=f'Mean = {mean_err*1000:.2f} mm')
axes[1].fill_between(trials,
                     (mean_err - std_err) * 1000,
                     (mean_err + std_err) * 1000,
                     color='red', alpha=0.1, label=f'±1σ = {std_err*1000:.2f} mm')
axes[1].set_ylabel('Norm Error (mm)')
axes[1].set_title(f'Position Norm Error  (n={n},  mean={mean_err*1000:.2f} mm,  std={std_err*1000:.2f} mm)')
axes[1].legend()
axes[1].grid(True)

# 3. 회전 축별 오차
axes[2].plot(trials, np.degrees(df['err_r']), marker='o', label='eR (roll)')
axes[2].plot(trials, np.degrees(df['err_p']), marker='s', label='eP (pitch)')
axes[2].plot(trials, np.degrees(df['err_y']), marker='^', label='eY (yaw)')
axes[2].axhline(0, color='black', linewidth=0.5, linestyle='--')
axes[2].set_ylabel('Error (deg)')
axes[2].set_title('IK Accuracy — Rotation Per-Axis Error')
axes[2].legend()
axes[2].grid(True)

# 4. 회전 Norm 오차
axes[3].bar(trials, np.degrees(df['rot_norm_err']), color='darkorange', label='Rot Norm Error')
axes[3].axhline(np.degrees(mean_rot_err), color='red', linewidth=1.5, linestyle='--',
                label=f'Mean = {np.degrees(mean_rot_err):.4f} deg')
axes[3].fill_between(trials,
                     np.degrees(mean_rot_err - std_rot_err),
                     np.degrees(mean_rot_err + std_rot_err),
                     color='red', alpha=0.1, label=f'±1σ = {np.degrees(std_rot_err):.4f} deg')
axes[3].set_xlabel('Trial')
axes[3].set_ylabel('Rot Norm Error (deg)')
axes[3].set_title(f'Rotation Norm Error  (n={n},  mean={np.degrees(mean_rot_err):.4f} deg,  std={np.degrees(std_rot_err):.4f} deg)')
axes[3].legend()
axes[3].grid(True)

plt.tight_layout()

png_name = "ik_accuracy.png"
png_path = os.path.join(PLOT_DIR, png_name)
plt.savefig(png_path, dpi=150)
print(f"Plot saved : {png_path}")

plt.show()

shutil.move(CSV_PATH, os.path.join(DONE_DIR, "ik_accuracy_log.csv"))
print(f"CSV moved  : {DONE_DIR}/ik_accuracy_log.csv")
