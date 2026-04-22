import pandas as pd
import matplotlib.pyplot as plt
import glob

# 최신 로그 자동 선택
# files = sorted(glob.glob("log_*.csv"))
files = sorted(glob.glob("*.csv"))
if not files:
    print("ERROR: log_*.csv 파일이 없습니다. 로봇 애플리케이션을 먼저 실행하세요.")
    exit(1)

print(f"Loading: {files[-1]}")
df = pd.read_csv(files[-1])

# timestamp를 초 단위로 변환
df['t'] = (df['timestamp_ns'] - df['timestamp_ns'].iloc[0]) * 1e-9

fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

# 관절 위치
for i in range(6):
    axes[0].plot(df['t'], df[f'q{i}'], label=f'q{i}')
axes[0].set_ylabel('Position (rad)')
axes[0].legend()

# 토크
for i in range(6):
    axes[1].plot(df['t'], df[f'tau{i}'], label=f'τ{i}')
axes[1].set_ylabel('Torque (Nm)')
axes[1].legend()

# TCP 궤적
axes[2].plot(df['t'], df['tcp_x'], label='X')
axes[2].plot(df['t'], df['tcp_y'], label='Y')
axes[2].plot(df['t'], df['tcp_z'], label='Z')
axes[2].set_ylabel('TCP Position (m)')
axes[2].set_xlabel('Time (s)')
axes[2].legend()

plt.tight_layout()
plt.savefig(files[-1].replace('.csv', '.png'), dpi=150)
plt.show()
