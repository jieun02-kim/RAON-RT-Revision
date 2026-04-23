import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import sys

CSV_PATH = os.path.join(os.path.dirname(__file__), "../ik_accuracy_log.csv")

if not os.path.exists(CSV_PATH):
    print(f"ERROR: {CSV_PATH} not found. Run IK trials first.")
    sys.exit(1)

df = pd.read_csv(CSV_PATH)
n = len(df)
print(f"Loaded {n} trials from {CSV_PATH}")
print(df[['err_x','err_y','err_z','norm_err']].describe())

trials = np.arange(1, n + 1)
mean_err = df['norm_err'].mean()
std_err  = df['norm_err'].std()

fig, axes = plt.subplots(2, 1, figsize=(10, 8))

# --- 상단: 축별 오차 ---
axes[0].plot(trials, df['err_x'] * 1000, marker='o', label='eX')
axes[0].plot(trials, df['err_y'] * 1000, marker='s', label='eY')
axes[0].plot(trials, df['err_z'] * 1000, marker='^', label='eZ')
axes[0].axhline(0, color='black', linewidth=0.5, linestyle='--')
axes[0].set_ylabel('Error (mm)')
axes[0].set_title('IK Accuracy — Per-Axis Error')
axes[0].legend()
axes[0].grid(True)

# --- 하단: Norm 오차 + 평균 ---
axes[1].bar(trials, df['norm_err'] * 1000, color='steelblue', label='Norm Error')
axes[1].axhline(mean_err * 1000, color='red', linewidth=1.5, linestyle='--',
                label=f'Mean = {mean_err*1000:.2f} mm')
axes[1].fill_between(trials,
                     (mean_err - std_err) * 1000,
                     (mean_err + std_err) * 1000,
                     color='red', alpha=0.1, label=f'±1σ = {std_err*1000:.2f} mm')
axes[1].set_xlabel('Trial')
axes[1].set_ylabel('Norm Error (mm)')
axes[1].set_title(f'IK Accuracy — Norm Error  (n={n}, mean={mean_err*1000:.2f}mm, std={std_err*1000:.2f}mm)')
axes[1].legend()
axes[1].grid(True)

plt.tight_layout()

os.makedirs(os.path.join(os.path.dirname(__file__), "plots"), exist_ok=True)
out_path = os.path.join(os.path.dirname(__file__), "plots", "ik_accuracy.png")
plt.savefig(out_path, dpi=150)
print(f"Plot saved: {out_path}")
plt.show()
