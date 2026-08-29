import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation

# rPc values: [tx, ty, tz, tux, tuy, tuz]
tx, ty, tz = -0.030688, 0.0463749, -0.0533527
tux, tuy, tuz = 2.21612, -0.00103439, 2.17288

theta = np.sqrt(tux**2 + tuy**2 + tuz**2)
axis = np.array([tux, tuy, tuz]) / theta
rotvec = axis * theta
R = Rotation.from_rotvec(rotvec).as_matrix()

def draw_frame(ax, R, t, label, scale=0.15, colors=('r','g','b')):
    axes = R.T  # columns = x, y, z axes
    for i, (color, name) in enumerate(zip(colors, ['X','Y','Z'])):
        ax.quiver(t[0], t[1], t[2],
                  axes[0,i]*scale, axes[1,i]*scale, axes[2,i]*scale,
                  color=color, linewidth=2)
    ax.text(t[0], t[1], t[2]+0.05, label, fontsize=11, fontweight='bold')

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Robot base frame (identity)
R_base = np.eye(3)
t_base = np.zeros(3)
draw_frame(ax, R_base, t_base, 'Robot Base')

# Camera frame
t_cam = np.array([tx, ty, tz])
draw_frame(ax, R, t_cam, 'Camera')

# Arrow from base to camera
ax.quiver(0, 0, 0, tx, ty, tz, color='purple', linewidth=1.5,
          linestyle='dashed', arrow_length_ratio=0.15)

# Print info
rpy = Rotation.from_matrix(R).as_euler('xyz', degrees=True)
print(f"Translation (m): X={tx:.4f}, Y={ty:.4f}, Z={tz:.4f}")
print(f"Rotation RPY (deg): R={rpy[0]:.2f}, P={rpy[1]:.2f}, Y={rpy[2]:.2f}")
print(f"Distance base→camera: {np.linalg.norm(t_cam)*100:.1f} cm")

ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('rMc: Robot Base → Camera\n(R=red X, G=green Y, B=blue Z)')
ax.set_xlim(-0.3, 0.3)
ax.set_ylim(-0.3, 0.3)
ax.set_zlim(-0.3, 0.3)

# Legend
from matplotlib.lines import Line2D
legend = [Line2D([0],[0],color='r',label='X axis'),
          Line2D([0],[0],color='g',label='Y axis'),
          Line2D([0],[0],color='b',label='Z axis'),
          Line2D([0],[0],color='purple',linestyle='dashed',label='base→cam')]
ax.legend(handles=legend)

plt.tight_layout()
plt.savefig('/home/raimlab/RAON-RT/App/CalibUtils/rPc_visualization.png', dpi=150)
plt.show()
print("Saved: rPc_visualization.png")
