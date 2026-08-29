#!/usr/bin/env python3
"""
Indy7 Control UI
Panels: [Control] | [RealSense Camera] | [Plots]

pip install PyQt5 matplotlib pandas numpy
pip install pyrealsense2   # optional
"""
import sys, os, struct, socket, threading
from datetime import datetime

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.cm as cm
import matplotlib.lines
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

import glob, subprocess
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QSplitter,
    QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QPushButton, QLabel, QLineEdit, QTextEdit, QStatusBar,
    QTabWidget, QFileDialog, QScrollArea, QSizePolicy, QMessageBox,
)
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal
from PyQt5.QtGui import QFont, QPixmap, QImage

try:
    import pyrealsense2 as rs
    _RS = True
except ImportError:
    _RS = False

# ═══════════════════════════════════════════════════════════════════════════════
# Protocol
# ═══════════════════════════════════════════════════════════════════════════════
_STX = bytes([0x02, 0x5B])
_ETX = bytes([0x5D, 0x03])
_MIN_PKT = 11

CMD_CONTROL = 0x43;  CMD_STATE = 0x53
CMD_ROBOT   = 0x82;  CMD_AXIS  = 0x65;  CMD_ECAT_MASTER = 0x77
SUBCMD_CTRL_SET_MODE        = 0x01;  SUBCMD_CTRL_TRIGGER_LOG     = 0x02
SUBCMD_CTRL_VS_TOGGLE       = 0x03;  SUBCMD_CTRL_SAVE_POSE       = 0x04
SUBCMD_CTRL_SET_TARGET_POSE = 0x05;  SUBCMD_CTRL_HOME            = 0x06
SUBCMD_CTRL_VS_REINIT       = 0x07
SUBCMD_STATE_QUERY = 0x01;  SUBCMD_STATE_PUSH = 0x02
SUBCMD_GET_METADATA = 0x01
_PRECISION  = 1000
_AXIS_TYPES = {0: 'Revolute', 1: 'Linear', 2: 'Jointless'}
_COMM_TYPES = {0: 'EtherCAT', 1: 'Virtual'}

CTRL_MODE_GRAV = 0x00;  CTRL_MODE_FULL = 0x01;  CTRL_MODE_CTC = 0x02
CTRL_MODE_IK   = 0x03;  CTRL_MODE_IK6  = 0x04

_MODE_NAMES = {0: 'Gravity Comp', 1: 'Full Dynamics', 2: 'Comp. Torque',
               3: 'IK  (i)', 4: 'IK 6DOF  (m)'}
_VS_NAMES   = {0: 'IDLE', 1: 'TRACKING', 2: 'TAG_LOST', 3: 'SINGULARITY'}
_VS_COLORS  = {0: '#888888', 1: '#00cc44', 2: '#ff8800', 3: '#ff2222'}

def _crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF

_spin = 0
def _make_pkt(cmd: int, subcmd: int, data: bytes = b'') -> bytes:
    global _spin
    sid = _spin & 0xFF;  _spin = (_spin + 1) & 0xFF
    n = len(data) + 2
    hdr = bytes([sid, n >> 8, n & 0xFF, cmd, subcmd]) + data
    c = _crc16(hdr)
    return _STX + hdr + bytes([c >> 8, c & 0xFF]) + _ETX

def _parse(buf: bytes):
    pkts = []
    while True:
        i = buf.find(_STX)
        if i < 0: buf = b''; break
        if i: buf = buf[i:]
        if len(buf) < _MIN_PKT: break
        n = buf[3] * 256 + buf[4];  total = n + 9
        if len(buf) < total: break
        if buf[total - 2] != 0x5D or buf[total - 1] != 0x03:
            buf = buf[2:]; continue
        pkts.append((buf[5], buf[6], buf[7:total - 4]))
        buf = buf[total:]
    return pkts, buf

def _decode_robot_meta(p: bytes) -> dict:
    if len(p) < 5: return {}
    name_len = p[4]
    name = p[5:5+name_len].decode('ascii', errors='replace') if len(p) >= 5+name_len else '?'
    return dict(is_sim=bool(p[0]), total_axes=(p[1]<<8)|p[2],
                ecat_enabled=bool(p[3]), name=name)

def _decode_ecat_meta(p: bytes) -> dict:
    if len(p) < 5: return {}
    return dict(master_state=p[0], slave_state=p[1], domain_state=p[2],
                slave_count=(p[3]<<8)|p[4])

def _decode_axis_meta_all(p: bytes) -> list:
    if len(p) < 1: return []
    axes, i = [], 1  # skip ALL_AXIS marker byte
    while i + 27 < len(p):
        ax_type  = p[i+1]
        pos_u = struct.unpack_from('>i', p, i+3)[0]  / _PRECISION
        pos_l = struct.unpack_from('>i', p, i+7)[0]  / _PRECISION
        vel_u = struct.unpack_from('>i', p, i+11)[0] / _PRECISION
        vel_l = struct.unpack_from('>i', p, i+15)[0] / _PRECISION
        acc_u = struct.unpack_from('>i', p, i+19)[0] / _PRECISION
        acc_l = struct.unpack_from('>i', p, i+23)[0] / _PRECISION
        name_len = p[i+27]
        name = p[i+28:i+28+name_len].decode('ascii', errors='replace')
        axes.append(dict(no=p[i], type=ax_type, comm=p[i+2],
                         pos_u=pos_u, pos_l=pos_l,
                         vel_u=vel_u, vel_l=vel_l,
                         acc_u=acc_u, acc_l=acc_l,
                         pos_unit='°' if ax_type == 0 else 'm', name=name))
        i += 28 + name_len
    return axes

def _decode_state(p: bytes):
    if len(p) < 51: return None
    x, y, z, r, pi, ya = struct.unpack_from('<6d', p, 3)
    return dict(mode=p[0], vs=p[1], logging=p[2],
                x=x, y=y, z=z, roll=r, pitch=pi, yaw=ya)

# ═══════════════════════════════════════════════════════════════════════════════
# Network thread
# ═══════════════════════════════════════════════════════════════════════════════
class _RecvThread(QThread):
    state_rx   = pyqtSignal(dict)
    log_msg    = pyqtSignal(str)
    conn_ok    = pyqtSignal()
    conn_fail  = pyqtSignal(str)
    conn_lost  = pyqtSignal()
    meta_robot = pyqtSignal(dict)
    meta_ecat  = pyqtSignal(dict)
    meta_axes  = pyqtSignal(list)

    def __init__(self, host: str, port: int):
        super().__init__()
        self.host, self.port = host, port
        self._sock = None;  self._running = False
        self._lock = threading.Lock()

    def run(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(3.0);  s.connect((self.host, self.port));  s.settimeout(0.1)
            with self._lock: self._sock = s
            self._running = True;  self.conn_ok.emit()
        except Exception as e:
            self.conn_fail.emit(str(e));  return
        buf = b''
        while self._running:
            try:
                chunk = self._sock.recv(4096)
                if not chunk: break
                buf += chunk
                pkts, buf = _parse(buf)
                for cmd, sub, payload in pkts:
                    if cmd == CMD_STATE and sub == SUBCMD_STATE_PUSH:
                        st = _decode_state(payload)
                        if st: self.state_rx.emit(st)
                    elif cmd == CMD_ROBOT and sub == SUBCMD_GET_METADATA:
                        m = _decode_robot_meta(payload)
                        if m: self.meta_robot.emit(m)
                    elif cmd == CMD_ECAT_MASTER and sub == SUBCMD_GET_METADATA:
                        m = _decode_ecat_meta(payload)
                        if m: self.meta_ecat.emit(m)
                    elif cmd == CMD_AXIS and sub == SUBCMD_GET_METADATA:
                        axes = _decode_axis_meta_all(payload)
                        if axes: self.meta_axes.emit(axes)
            except socket.timeout: continue
            except Exception as e:
                if self._running: self.log_msg.emit(f"Recv: {e}")
                break
        self.conn_lost.emit()

    def send(self, data: bytes):
        with self._lock:
            if self._sock:
                try: self._sock.sendall(data)
                except Exception as e: self.log_msg.emit(f"Send: {e}")

    def stop(self):
        self._running = False
        with self._lock:
            if self._sock:
                try: self._sock.close()
                except: pass
                self._sock = None

# ═══════════════════════════════════════════════════════════════════════════════
# Camera thread (RealSense)
# ═══════════════════════════════════════════════════════════════════════════════
class _CamThread(QThread):
    frame_ready = pyqtSignal(object)   # np.ndarray BGR
    cam_error   = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._running = False
        self._last_frame = None  # 최신 프레임 (캘리브레이션 캡처용)

    def run(self):
        if not _RS:
            self.cam_error.emit("pyrealsense2 not installed");  return
        try:
            pipe = rs.pipeline()
            cfg  = rs.config()
            cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            pipe.start(cfg)
        except Exception as e:
            self.cam_error.emit(str(e));  return
        self._running = True
        while self._running:
            try:
                frames = pipe.wait_for_frames(timeout_ms=500)
                cf = frames.get_color_frame()
                if cf:
                    frame = np.asanyarray(cf.get_data()).copy()  # 버퍼 소유권 확보
                    self._last_frame = frame
                    self.frame_ready.emit(frame)
            except Exception as e:
                self.cam_error.emit(f"loop: {e}"); break
        try: pipe.stop()
        except: pass

    def stop(self):
        self._running = False

# ═══════════════════════════════════════════════════════════════════════════════
# Subprocess thread (calibration steps)
# ═══════════════════════════════════════════════════════════════════════════════
class _ProcThread(QThread):
    line_ready = pyqtSignal(str)
    proc_done  = pyqtSignal(int)   # exit code

    def __init__(self, cmd: list, cwd: str):
        super().__init__()
        self._cmd = cmd
        self._cwd = cwd

    def run(self):
        try:
            proc = subprocess.Popen(self._cmd, stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, cwd=self._cwd, text=True)
            for line in proc.stdout:
                self.line_ready.emit(line.rstrip())
            proc.wait()
            self.proc_done.emit(proc.returncode)
        except Exception as e:
            self.line_ready.emit(f"[error] {e}")
            self.proc_done.emit(-1)

# ═══════════════════════════════════════════════════════════════════════════════
# Plot functions (adapted from existing scripts)
# ═══════════════════════════════════════════════════════════════════════════════
_GOAL_COLS   = ['goal_x', 'goal_y', 'goal_z', 'goal_roll', 'goal_pitch', 'goal_yaw']
_ORIENT_COLS = ['tcp_roll', 'tcp_pitch', 'tcp_yaw']
_TAIL = 500

def _rpy_rot(r, p, y):
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    return np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                     [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
                     [-sp,   cp*sr,             cp*cr]])

def _rot_err_deg(rpy_g, rpy_a):
    R = _rpy_rot(*rpy_g) @ _rpy_rot(*rpy_a).T
    return np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))

def _plot_tcp(paths: list) -> tuple:
    """Returns (fig_traj, fig_err). Adapted from rt_log_results/plot_tcp_trajectory.py."""
    datasets = []
    for p in paths:
        df = pd.read_csv(p)
        df['t'] = (df['timestamp_ns'] - df['timestamp_ns'].iloc[0]) * 1e-9
        label = os.path.splitext(os.path.basename(p))[0].replace("DataLog_", "")
        datasets.append((label, df))
    colors = cm.tab10(np.linspace(0, 1, max(len(datasets), 2)))

    # ── Figure 1: trajectories ────────────────────────────────────────────────
    fig1 = Figure(figsize=(16, 7))
    gs   = fig1.add_gridspec(2, 3, height_ratios=[3, 2], hspace=0.42, wspace=0.32)
    ax1  = fig1.add_subplot(gs[0, :2], projection='3d')   # 3D 크게 (좌 2/3)
    ax3  = fig1.add_subplot(gs[0, 2])                      # XY projection (우 1/3)
    ax2  = fig1.add_subplot(gs[1, :])                      # TCP vs Time (하단 전체폭)

    for (lbl, df), c in zip(datasets, colors):
        x, y, z, t = df['tcp_x'], df['tcp_y'], df['tcp_z'], df['t']
        ax1.plot(x, y, z, lw=1.4, color=c, label=lbl)
        ax1.scatter(x.iloc[0],  y.iloc[0],  z.iloc[0],  c=[c], s=40, marker='o')
        ax1.scatter(x.iloc[-1], y.iloc[-1], z.iloc[-1], c=[c], s=40, marker='x')
        ax2.plot(t, x, lw=1, color=c, ls='-')
        ax2.plot(t, y, lw=1, color=c, ls='--')
        ax2.plot(t, z, lw=1, color=c, ls=':')
        ax3.plot(x, y, lw=1.2, color=c, label=lbl)
        ax3.scatter(x.iloc[0],  y.iloc[0],  c=[c], s=30, marker='o')
        ax3.scatter(x.iloc[-1], y.iloc[-1], c=[c], s=30, marker='x')

    ax1.set_xlabel('X(m)', fontsize=9); ax1.set_ylabel('Y(m)', fontsize=9)
    ax1.set_zlabel('Z(m)', fontsize=9); ax1.set_title('3D Trajectory', fontsize=11)
    ax1.legend(fontsize=7)
    ax2.set_xlabel('Time(s)', fontsize=8); ax2.set_ylabel('Pos(m)', fontsize=8)
    ax2.set_title('TCP vs Time', fontsize=9)
    lh = [matplotlib.lines.Line2D([0],[0], color='k', ls=s, label=n)
          for s, n in (('-','X'), ('--','Y'), (':','Z'))]
    lh += [matplotlib.lines.Line2D([0],[0], color=c, lw=1.5, label=l)
           for (l, _), c in zip(datasets, colors)]
    ax2.legend(handles=lh, fontsize=6)
    ax3.set_xlabel('X(m)', fontsize=8); ax3.set_ylabel('Y(m)', fontsize=8)
    ax3.set_title('XY Projection', fontsize=9)
    ax3.legend(fontsize=6); ax3.set_aspect('equal')

    # ── Figure 2: per-run error ───────────────────────────────────────────────
    fig2    = Figure(figsize=(14, 3.5))
    ax_p    = fig2.add_subplot(121)
    ax_r    = fig2.add_subplot(122)
    fig2.suptitle('Per-Run Error (last 500 rows = settled state)', fontsize=10)

    HG  = all(c in datasets[0][1].columns for c in _GOAL_COLS) if datasets else False
    fps = np.array([df.tail(_TAIL)[['tcp_x','tcp_y','tcp_z']].mean().values for _, df in datasets])
    mfp = fps.mean(axis=0)
    labels_e, pe, re = [], [], []

    for (lbl, df), fp in zip(datasets, fps):
        tail = df.tail(_TAIL);  labels_e.append(lbl)
        hg = HG and all(c in df.columns for c in _GOAL_COLS + _ORIENT_COLS)
        if hg:
            gp = tail[_GOAL_COLS[:3]].mean().values
            if np.linalg.norm(gp) > 1e-4:
                pe.append(np.linalg.norm(fp - gp) * 1000)
                re.append(_rot_err_deg(tail[_GOAL_COLS[3:]].mean().values,
                                       tail[_ORIENT_COLS].mean().values))
                continue
        pe.append(np.linalg.norm(fp - mfp) * 1000);  re.append(np.nan)

    xi = np.arange(len(labels_e));  bw = 0.5
    use_abs = any(not np.isnan(v) for v in re)
    bars = ax_p.bar(xi, pe, width=bw, color=colors[:len(labels_e)], edgecolor='black', lw=0.5)
    ax_p.set_xticks(xi);  ax_p.set_xticklabels(labels_e, rotation=25, ha='right', fontsize=7)
    ax_p.set_ylabel('Error (mm)')
    ax_p.set_title('||TCP − Goal|| (mm)' if use_abs else 'Deviation from mean (mm)')
    ax_p.yaxis.grid(True, ls='--', alpha=0.5);  ax_p.set_axisbelow(True)
    for bar, val in zip(bars, pe):
        if not np.isnan(val):
            ax_p.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.01,
                      f'{val:.3f}', ha='center', va='bottom', fontsize=7)

    if use_abs:
        bars2 = ax_r.bar(xi, re, width=bw, color=colors[:len(labels_e)], edgecolor='black', lw=0.5)
        ax_r.set_xticks(xi);  ax_r.set_xticklabels(labels_e, rotation=25, ha='right', fontsize=7)
        ax_r.set_ylabel('Orientation Error (deg)')
        ax_r.set_title('angle(R_final, R_goal) (deg)')
        ax_r.yaxis.grid(True, ls='--', alpha=0.5);  ax_r.set_axisbelow(True)
        for bar, val in zip(bars2, re):
            if not np.isnan(val):
                ax_r.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.001,
                          f'{val:.4f}', ha='center', va='bottom', fontsize=7)
    else:
        ax_r.text(0.5, 0.5, 'No goal pose in CSV.\nRun IK trial with target set first.',
                  ha='center', va='center', transform=ax_r.transAxes, fontsize=9, color='gray')
        ax_r.set_title('Orientation Error (deg)');  ax_r.set_xticks([]);  ax_r.set_yticks([])
    fig2.tight_layout()
    return fig1, fig2



# ═══════════════════════════════════════════════════════════════════════════════
# Main window
# ═══════════════════════════════════════════════════════════════════════════════
_RT_LOG_DIR   = os.path.join(os.path.dirname(__file__), 'rt_log_results')
_CALIB_ROOT   = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'CalibUtils'))
_CALIB_ETH    = os.path.join(_CALIB_ROOT, 'eye_to_hand')    # eye-to-hand 데이터/결과
_CALIB_EIH    = os.path.join(_CALIB_ROOT, 'eye_in_hand')    # eye-in-hand 데이터/결과

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Indy7 Control UI")
        self.resize(1440, 960)

        self._recv: _RecvThread | None = None
        self._cam:  _CamThread  | None = None
        self._last_vs  = 0
        self._meta_robot: dict = {}
        self._meta_ecat:  dict = {}
        self._meta_axes:  list = []
        self._traj_paths:   list[str] = []
        self._calib_count:  int = 0         # 현재 세션에서 캡처한 수
        self._proc_thread: _ProcThread | None = None
        self._last_state:   dict | None = None  # 최근 수신 로봇 상태
        self._traj_figs: list = []
        self._cam_frames = 0

        self._poll = QTimer()
        self._poll.timeout.connect(self._query_state)
        self._fps_timer = QTimer()
        self._fps_timer.timeout.connect(self._update_fps)
        self._fps_timer.setInterval(1000)

        self._build_ui()
        self._start_camera()

    # ──────────────────────────────────────────────────────────────────────────
    # Layout builders
    # ──────────────────────────────────────────────────────────────────────────
    def _build_ui(self):
        central = QWidget();  self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setContentsMargins(4, 4, 4, 4);  root.setSpacing(4)

        # 왼쪽: 컨트롤 패널 (풀 높이)
        root.addWidget(self._make_ctrl_panel())

        # 오른쪽: 위=카메라 / 아래=플롯
        right_spl = QSplitter(Qt.Vertical)
        right_spl.addWidget(self._make_cam_panel())
        right_spl.addWidget(self._make_plot_panel())
        right_spl.setSizes([560, 380])
        root.addWidget(right_spl, stretch=1)

        self.setStatusBar(QStatusBar())
        self.statusBar().showMessage("Disconnected")

    # ── Left: control ─────────────────────────────────────────────────────────
    def _make_ctrl_panel(self):
        w = QWidget();  w.setMinimumWidth(360);  w.setMaximumWidth(420)
        v = QVBoxLayout(w);  v.setSpacing(5);  v.setContentsMargins(2, 2, 2, 2)

        # Connection
        cbox = QGroupBox("Connection");  crow = QHBoxLayout(cbox)
        self._e_host = QLineEdit("localhost");  self._e_host.setFixedWidth(108)
        self._e_port = QLineEdit("7420");       self._e_port.setFixedWidth(48)
        self._b_conn = QPushButton("Connect");  self._b_conn.setFixedWidth(78)
        self._b_conn.clicked.connect(self._toggle_conn)
        for ww in (QLabel("Host:"), self._e_host, QLabel("Port:"), self._e_port, self._b_conn):
            crow.addWidget(ww)
        crow.addStretch();  v.addWidget(cbox)

        # Dynamics-only modes
        dbox = QGroupBox("Dynamics Control");  dl = QVBoxLayout(dbox)
        self._mode_btns = []
        for lbl, code in [("Gravity Comp",  CTRL_MODE_GRAV),
                           ("Full Dynamics", CTRL_MODE_FULL),
                           ("Comp. Torque",  CTRL_MODE_CTC)]:
            b = QPushButton(lbl);  b.setEnabled(False);  b.setMinimumHeight(30)
            b.clicked.connect(lambda _, c=code: self._set_mode(c))
            dl.addWidget(b);  self._mode_btns.append(b)
        v.addWidget(dbox)

        # Kinematics + dynamics modes
        kbox = QGroupBox("Kinematics Control");  kl = QVBoxLayout(kbox)

        # Target pose input grid
        tg = QGridLayout();  tg.setSpacing(2)
        self._target_edits = {}
        for row, (key, default) in enumerate([('x','0.0'), ('y','0.0'), ('z','0.5'),
                                              ('roll','0.0'), ('pitch','0.0'), ('yaw','0.0')]):
            unit = 'm' if key in ('x','y','z') else 'rad'
            e = QLineEdit(default)
            e.setFixedWidth(144);  e.setFont(QFont("Monospace", 9))
            tg.addWidget(QLabel(f"{key}:"),  row, 0)
            tg.addWidget(e,                  row, 1)
            tg.addWidget(QLabel(unit),       row, 2)
            self._target_edits[key] = e
        kl.addLayout(tg)

        # Set / Fill buttons
        br = QHBoxLayout()
        self._b_set_target  = QPushButton("Set Target")
        self._b_fill_target = QPushButton("Fill from TCP")
        self._b_set_target.setEnabled(False)
        self._b_fill_target.setEnabled(False)
        self._b_set_target.clicked.connect(self._send_target_pose)
        self._b_fill_target.clicked.connect(self._fill_target_from_tcp)
        br.addWidget(self._b_set_target);  br.addWidget(self._b_fill_target)
        kl.addLayout(br)

        kl.addSpacing(4)
        for lbl, code in [("IK  (i)",      CTRL_MODE_IK),
                           ("IK 6DOF  (m)", CTRL_MODE_IK6)]:
            b = QPushButton(lbl);  b.setEnabled(False);  b.setMinimumHeight(30)
            b.clicked.connect(lambda _, c=code: self._set_mode(c))
            kl.addWidget(b);  self._mode_btns.append(b)
        v.addWidget(kbox)

        # Tools
        tbox = QGroupBox("Tools");  tl = QVBoxLayout(tbox)
        self._b_home = QPushButton("Home  (h)")
        self._b_vs   = QPushButton("Visual Servo  (v)");  self._b_vs.setCheckable(True)
        self._b_save = QPushButton("Save Target Pose  (s)")
        self._b_log  = QPushButton("Trigger Log  (l)")
        for b in (self._b_home, self._b_vs, self._b_save, self._b_log):
            b.setEnabled(False);  b.setMinimumHeight(30);  tl.addWidget(b)
        self._b_home.clicked.connect(self._go_home)
        self._b_vs.clicked.connect(self._toggle_vs)
        self._b_save.clicked.connect(self._save_pose)
        self._b_log.clicked.connect(self._trigger_log)
        v.addWidget(tbox)

        # Robot state
        sbox = QGroupBox("Robot State");  sg = QGridLayout(sbox)
        self._l_mode = self._blbl();  self._l_vs = self._blbl();  self._l_log = self._blbl()
        for row, (txt, lbl) in enumerate([("Mode:", self._l_mode),
                                           ("VS:",   self._l_vs),
                                           ("Log:",  self._l_log)]):
            sg.addWidget(QLabel(txt), row, 0);  sg.addWidget(lbl, row, 1)
        v.addWidget(sbox)

        # TCP pose
        pbox = QGroupBox("TCP Pose");  pg = QGridLayout(pbox)
        pg.setColumnMinimumWidth(1, 88)
        self._pose_lbls = {}
        for row, (key, unit) in enumerate([('x','m'),('y','m'),('z','m'),
                                            ('roll','rad'),('pitch','rad'),('yaw','rad')]):
            lbl = QLabel("—")
            lbl.setFont(QFont("Monospace", 10))
            lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            pg.addWidget(QLabel(f"{key}:"), row, 0)
            pg.addWidget(lbl, row, 1)
            pg.addWidget(QLabel(unit), row, 2)
            self._pose_lbls[key] = lbl
        v.addWidget(pbox)

        # Status log — stretch=1 so it fills remaining vertical space
        lgbox = QGroupBox("Status Log");  lgl = QVBoxLayout(lgbox)
        self._log_w = QTextEdit()
        self._log_w.setReadOnly(True)
        self._log_w.setMinimumHeight(120)
        self._log_w.setFont(QFont("Monospace", 8))
        lgl.addWidget(self._log_w)
        v.addWidget(lgbox, stretch=1)
        return w

    # ── Center: camera ────────────────────────────────────────────────────────
    def _make_cam_panel(self):
        w = QWidget()
        outer = QHBoxLayout(w)
        outer.setContentsMargins(2, 2, 2, 2);  outer.setSpacing(0)

        cambox = QGroupBox("RealSense Camera")
        cl = QVBoxLayout(cambox)

        self._cam_label = QLabel("Camera not started")
        self._cam_label.setAlignment(Qt.AlignCenter)
        self._cam_label.setFixedSize(640, 480)
        self._cam_label.setStyleSheet("background-color:#111; color:#777;")
        self._cam_label.setFont(QFont("Monospace", 11))
        cl.addWidget(self._cam_label)

        # Info row: VS state + FPS
        info = QHBoxLayout()
        self._cam_vs_lbl  = QLabel("VS: —");  self._cam_vs_lbl.setFont(QFont("Monospace", 9))
        self._cam_fps_lbl = QLabel("");        self._cam_fps_lbl.setFont(QFont("Monospace", 9))
        info.addWidget(self._cam_vs_lbl);  info.addStretch();  info.addWidget(self._cam_fps_lbl)
        cl.addLayout(info)

        note = QLabel("※ Camera auto-stops when VS activates (shared device) and resumes when VS stops.")
        note.setFont(QFont("Sans", 8));  note.setStyleSheet("color:#888;");  note.setWordWrap(True)
        cl.addWidget(note)

        outer.addWidget(cambox)
        outer.addStretch()          # 카메라 오른쪽 여백 → 좌정렬
        return w

    # ── Right: plots ──────────────────────────────────────────────────────────
    def _make_plot_panel(self):
        tabs = QTabWidget()

        # ── Tab 1: Data Log ───────────────────────────────────────────────────
        t1 = QWidget();  t1v = QVBoxLayout(t1);  t1v.setSpacing(4)

        traj_row = QHBoxLayout()
        self._b_traj_browse = QPushButton("Browse DataLog CSV…")
        self._b_traj_plot   = QPushButton("Plot");  self._b_traj_plot.setEnabled(False)
        self._b_traj_save   = QPushButton("Save PNG");  self._b_traj_save.setEnabled(False)
        self._b_traj_browse.clicked.connect(self._browse_traj)
        self._b_traj_plot.clicked.connect(self._do_traj_plot)
        self._b_traj_save.clicked.connect(lambda: self._save_png("traj"))
        for b in (self._b_traj_browse, self._b_traj_plot, self._b_traj_save):
            traj_row.addWidget(b)
        t1v.addLayout(traj_row)

        self._l_traj_files = QLabel("No files selected")
        self._l_traj_files.setFont(QFont("Monospace", 8));  self._l_traj_files.setWordWrap(True)
        t1v.addWidget(self._l_traj_files)

        self._traj_scroll = QScrollArea();  self._traj_scroll.setWidgetResizable(True)
        self._traj_inner  = QWidget()
        self._traj_vlay   = QVBoxLayout(self._traj_inner)
        self._traj_scroll.setWidget(self._traj_inner)
        t1v.addWidget(self._traj_scroll, stretch=1)
        tabs.addTab(t1, "Data Log")

        # ── Tab 3: Metadata ───────────────────────────────────────────────────
        t3 = QWidget();  t3v = QVBoxLayout(t3);  t3v.setSpacing(4)
        self._meta_text = QTextEdit()
        self._meta_text.setReadOnly(True)
        self._meta_text.setFont(QFont("Monospace", 9))
        self._meta_text.setPlaceholderText("Connect to robot to receive metadata…")
        t3v.addWidget(self._meta_text)
        tabs.addTab(t3, "Metadata")

        # ── Tab 4: Calibration ────────────────────────────────────────────────
        t4 = QWidget()
        t4_root = QHBoxLayout(t4);  t4_root.setSpacing(6);  t4_root.setContentsMargins(6,6,6,6)

        # ── 왼쪽: 단계 + 로그 ──────────────────────────────────────────────────
        left_w = QWidget();  t4v = QVBoxLayout(left_w);  t4v.setSpacing(6)

        # Step 1: Camera init
        grp1 = QGroupBox("Step 1 · Camera Init")
        g1l = QHBoxLayout(grp1)
        b_cam_init = QPushButton("Run save_camera_params  →  camera.xml")
        b_cam_init.clicked.connect(self._calib_save_cam)
        g1l.addWidget(b_cam_init)
        self._calib_proc_btns = [b_cam_init]
        t4v.addWidget(grp1)

        # Step 2: Capture
        grp2 = QGroupBox("Step 2 · Capture (pose + image)")
        g2v = QVBoxLayout(grp2)
        cap_row = QHBoxLayout()
        b_cap_start = QPushButton("Start New Session  (기존 파일 정리)")
        b_cap_start.clicked.connect(self._calib_start_session)
        self._b_cap_capture = QPushButton("Capture  #0")
        self._b_cap_capture.setEnabled(False)
        self._b_cap_capture.clicked.connect(self._calib_capture)
        cap_row.addWidget(b_cap_start);  cap_row.addWidget(self._b_cap_capture)
        g2v.addLayout(cap_row)
        self._calib_cap_log = QTextEdit()
        self._calib_cap_log.setReadOnly(True)
        self._calib_cap_log.setFont(QFont("Monospace", 8))
        self._calib_cap_log.setFixedHeight(70)
        g2v.addWidget(self._calib_cap_log)
        t4v.addWidget(grp2)

        # Step 3: Compute cPo
        grp3 = QGroupBox("Step 3 · Compute pose_cPo  (visp-compute-apriltag-poses)")
        g3l = QHBoxLayout(grp3)
        b_cpo = QPushButton("Run visp-compute-apriltag-poses")
        b_cpo.clicked.connect(self._calib_compute_cpo)
        g3l.addWidget(b_cpo)
        self._calib_proc_btns.append(b_cpo)
        t4v.addWidget(grp3)

        # Step 4: Solve + apply
        grp4 = QGroupBox("Step 4 · Solve AX=ZB  →  rPc.yaml")
        g4l = QHBoxLayout(grp4)
        b_solve = QPushButton("Run eye_to_hand_calib.py")
        b_apply = QPushButton("Apply to VS  (rPc.yaml 재로드)")
        b_solve.clicked.connect(self._calib_solve)
        b_apply.clicked.connect(self._calib_apply)
        g4l.addWidget(b_solve);  g4l.addWidget(b_apply)
        self._calib_proc_btns.append(b_solve)
        t4v.addWidget(grp4)

        # Output log
        self._calib_log = QTextEdit()
        self._calib_log.setReadOnly(True)
        self._calib_log.setFont(QFont("Monospace", 8))
        t4v.addWidget(self._calib_log, stretch=1)

        t4_root.addWidget(left_w, stretch=1)

        # ── 오른쪽: 시각화 ─────────────────────────────────────────────────────
        right_w = QWidget();  rv = QVBoxLayout(right_w);  rv.setSpacing(4)
        viz_hdr = QHBoxLayout()
        viz_hdr.addWidget(QLabel("rMc Visualization"))
        b_viz = QPushButton("Show / Refresh")
        b_viz.clicked.connect(self._calib_show_viz)
        viz_hdr.addStretch();  viz_hdr.addWidget(b_viz)
        rv.addLayout(viz_hdr)

        self._calib_viz_canvas = FigureCanvas(Figure(figsize=(5, 5)))
        rv.addWidget(self._calib_viz_canvas, stretch=1)

        t4_root.addWidget(right_w, stretch=1)

        tabs.addTab(t4, "Calibration")

        return tabs

    # ──────────────────────────────────────────────────────────────────────────
    # Calibration
    # ──────────────────────────────────────────────────────────────────────────
    def _calib_log_append(self, text: str):
        self._calib_log.append(text)
        self._calib_log.verticalScrollBar().setValue(self._calib_log.verticalScrollBar().maximum())

    def _calib_run_proc(self, cmd: list, cwd: str):
        """외부 프로그램을 백그라운드 스레드에서 실행하고 stdout을 calib_log에 스트리밍."""
        self._set_calib_btns_enabled(False)
        self._proc_thread = _ProcThread(cmd, cwd)
        self._proc_thread.line_ready.connect(self._calib_log_append)
        self._proc_thread.proc_done.connect(self._on_proc_done)
        self._proc_thread.start()

    def _on_proc_done(self, code: int):
        self._calib_log_append(f"[done] exit code {code}")
        self._set_calib_btns_enabled(True)

    def _set_calib_btns_enabled(self, enabled: bool):
        for b in self._calib_proc_btns:
            b.setEnabled(enabled)

    def _calib_save_cam(self):
        self._calib_log_append("▶ save_camera_params …")
        self._calib_run_proc([os.path.join(_CALIB_ROOT, 'save_camera_params'),
                              os.path.join(_CALIB_ETH, 'camera.xml')], cwd=_CALIB_ROOT)

    def _calib_start_session(self):
        """기존 캡처 파일 정리 후 새 세션 시작."""
        old = (glob.glob(os.path.join(_CALIB_ETH, 'pose_cPo_*.yaml')) +
               glob.glob(os.path.join(_CALIB_ETH, 'image*.png')))
        if old:
            reply = QMessageBox.question(
                self, "기존 파일 삭제",
                f"기존 캡처 파일 {len(old)}개를 삭제하고 새 세션을 시작할까요?\n"
                f"({_CALIB_ETH})",
                QMessageBox.Yes | QMessageBox.No)
            if reply != QMessageBox.Yes:
                return
            for f in old:
                os.remove(f)
            self._calib_log_append(f"[정리] {len(old)}개 파일 삭제 완료")

        self._calib_count = 0
        self._b_cap_capture.setEnabled(True)
        self._b_cap_capture.setText("Capture  #1")
        self._calib_cap_log.clear()
        self._calib_log_append("▶ 새 eye-to-hand 캘리브레이션 세션 시작")

    def _calib_capture(self):
        """현재 로봇 자세(rPe) + 카메라 프레임을 한 번에 저장."""
        from scipy.spatial.transform import Rotation as _Rot
        n = self._calib_count + 1

        st = self._last_state
        if st is None:
            QMessageBox.warning(self, "Capture", "로봇 상태를 수신하지 못했습니다. 먼저 연결하세요.")
            return

        roll, pitch, yaw = st['roll'], st['pitch'], st['yaw']
        tx, ty, tz = st['x'], st['y'], st['z']
        rotvec = _Rot.from_euler('xyz', [roll, pitch, yaw]).as_rotvec()

        # 카메라 프레임을 미리 복사해둠 (스레드 저장 중 변경 방지)
        frame_snapshot = (self._cam._last_frame.copy()
                          if self._cam and self._cam._last_frame is not None
                          else None)

        self._calib_count = n
        self._b_cap_capture.setText(f"Capture  #{n+1}")
        self._calib_log_append(f"[capture #{n}] 저장 중…")

        # 파일 I/O를 백그라운드 스레드에서 실행 → 메인 스레드 블로킹 없음
        def _save():
            import cv2
            rpe_path = os.path.join(_CALIB_ETH, f'pose_rPe_{n}.yaml')
            with open(rpe_path, 'w') as f:
                f.write("rows: 6\ncols: 1\ndata: \n")
                for v in [tx, ty, tz, *rotvec]:
                    f.write(f"  - [{v}]\n")

            img_ok = False
            if frame_snapshot is not None:
                img_path = os.path.join(_CALIB_ETH, f'image{n:04d}.png')
                cv2.imwrite(img_path, frame_snapshot)
                img_ok = True

            msg = (f"#{n}  rPe=({tx:.3f},{ty:.3f},{tz:.3f})  "
                   f"img={'저장됨' if img_ok else '카메라 없음'}")
            # UI 업데이트는 메인 스레드로 전달
            self._calib_cap_log.append(msg)
            self._calib_log_append(f"[capture #{n}] {msg}")

        t = threading.Thread(target=_save, daemon=True)
        t.start()

    def _calib_compute_cpo(self):
        self._calib_log_append("▶ visp-compute-apriltag-poses …")
        visp_bin = os.path.join(_CALIB_ROOT, 'visp-compute-apriltag-poses')
        cam_xml  = os.path.join(_CALIB_ETH, 'camera.xml')
        self._calib_run_proc([visp_bin,
                              '--input',  os.path.join(_CALIB_ETH, 'image%04d.png'),
                              '--intrinsic', cam_xml,
                              '--output', os.path.join(_CALIB_ETH, 'pose_cPo_%d.yaml'),
                              '--tag-size', '0.053'],
                             cwd=_CALIB_ETH)

    def _calib_solve(self):
        self._calib_log_append("▶ eye_to_hand_calib.py …")
        self._calib_run_proc(['python3',
                              os.path.join(_CALIB_ETH, 'eye_to_hand_calib.py')],
                             cwd=_CALIB_ETH)
        self._calib_show_viz()

    def _calib_show_viz(self):
        """rPc.yaml을 읽어 rMc 결과를 캔버스에 그린다."""
        from scipy.spatial.transform import Rotation as _Rot
        rpc_path = os.path.join(_CALIB_ETH, 'rPc.yaml')
        if not os.path.exists(rpc_path):
            self._calib_log_append("[viz] rPc.yaml 없음 — Step 4를 먼저 실행하세요.")
            return
        import yaml
        with open(rpc_path) as f:
            data = yaml.safe_load(f)
        vals = [v[0] for v in data['data']]
        tx, ty, tz = vals[0], vals[1], vals[2]
        rotvec = np.array(vals[3:6])
        R = _Rot.from_rotvec(rotvec).as_matrix()
        rpy = _Rot.from_rotvec(rotvec).as_euler('xyz', degrees=True)

        fig = self._calib_viz_canvas.figure
        fig.clear()
        ax = fig.add_subplot(111, projection='3d')

        def draw_frame(ax, R, t, label, scale=0.12):
            colors = ['r', 'g', 'b']
            for i, c in enumerate(colors):
                ax.quiver(t[0], t[1], t[2],
                          R[0,i]*scale, R[1,i]*scale, R[2,i]*scale,
                          color=c, linewidth=2)
            ax.text(t[0]+0.02, t[1]+0.02, t[2]+0.02, label, fontsize=9, fontweight='bold')

        draw_frame(ax, np.eye(3), np.zeros(3), 'Robot Base')
        t_cam = np.array([tx, ty, tz])
        draw_frame(ax, R, t_cam, 'Camera')
        ax.quiver(0, 0, 0, tx, ty, tz, color='purple', linewidth=1.5,
                  linestyle='dashed', arrow_length_ratio=0.12)

        rpe_files = sorted(glob.glob(os.path.join(_CALIB_ETH, 'pose_rPe_*.yaml')))
        for rf in rpe_files:
            with open(rf) as f2:
                d2 = yaml.safe_load(f2)
            v2 = [x[0] for x in d2['data']]
            ax.scatter(v2[0], v2[1], v2[2], c='orange', s=20)

        lim = max(np.linalg.norm(t_cam) * 1.5, 0.4)
        ax.set_xlim(-lim, lim);  ax.set_ylim(-lim, lim);  ax.set_zlim(-lim, lim)
        ax.set_xlabel('X');  ax.set_ylabel('Y');  ax.set_zlabel('Z')
        ax.set_title(
            f"Eye-to-Hand rMc  t=({tx:.3f}, {ty:.3f}, {tz:.3f})\n"
            f"RPY=({rpy[0]:.1f}°, {rpy[1]:.1f}°, {rpy[2]:.1f}°)",
            fontsize=9)
        fig.tight_layout()
        self._calib_viz_canvas.draw()
        self._calib_log_append(
            f"[viz] rMc: t=({tx:.4f}, {ty:.4f}, {tz:.4f})  "
            f"RPY=({rpy[0]:.2f}°, {rpy[1]:.2f}°, {rpy[2]:.2f}°)")

    def _calib_apply(self):
        """rPc.yaml 확인 후 VS Stop → Init(파일 재로드) → Start 명령 전송."""
        rpc_path = os.path.join(_CALIB_ETH, 'rPc.yaml')
        if not os.path.exists(rpc_path):
            QMessageBox.warning(self, "Apply", "rPc.yaml이 없습니다. Step 4를 먼저 실행하세요.")
            return

        import yaml
        with open(rpc_path) as f:
            data = yaml.safe_load(f)
        vals = [v[0] for v in data['data']]
        self._calib_log_append(
            f"[apply] rPc: t=({vals[0]:.4f}, {vals[1]:.4f}, {vals[2]:.4f})  "
            f"tu=({vals[3]:.4f}, {vals[4]:.4f}, {vals[5]:.4f})")

        if self._recv is None:
            QMessageBox.warning(self, "Apply", "로봇에 연결되지 않았습니다.")
            return

        self._send(CMD_CONTROL, SUBCMD_CTRL_VS_REINIT)
        self._calib_log_append("[apply] VS_REINIT 명령 전송 — 로봇이 rPc.yaml을 재로드합니다.")
        self._log("→ VS Re-Init (rPc.yaml 재로드)")

    def _blbl(self) -> QLabel:
        l = QLabel("—");  l.setFont(QFont("Monospace", 10, QFont.Bold));  return l

    # ──────────────────────────────────────────────────────────────────────────
    # Connection
    # ──────────────────────────────────────────────────────────────────────────
    def _toggle_conn(self):
        if self._recv is None:
            host = self._e_host.text().strip()
            port = int(self._e_port.text().strip())
            self._recv = _RecvThread(host, port)
            self._recv.state_rx.connect(self._on_state)
            self._recv.log_msg.connect(self._log)
            self._recv.conn_ok.connect(self._on_conn_ok)
            self._recv.conn_fail.connect(self._on_conn_fail)
            self._recv.conn_lost.connect(self._on_conn_lost)
            self._recv.meta_robot.connect(self._on_meta_robot)
            self._recv.meta_ecat.connect(self._on_meta_ecat)
            self._recv.meta_axes.connect(self._on_meta_axes)
            self._b_conn.setEnabled(False);  self._recv.start()
        else:
            self._recv.stop()

    def _on_conn_ok(self):
        self._b_conn.setText("Disconnect");  self._b_conn.setEnabled(True)
        self._e_host.setEnabled(False);  self._e_port.setEnabled(False)
        for b in self._mode_btns + [self._b_home, self._b_vs, self._b_save, self._b_log,
                                      self._b_set_target, self._b_fill_target]:
            b.setEnabled(True)
        self.statusBar().showMessage(f"Connected  ·  {self._e_host.text()}:{self._e_port.text()}")
        self._log(f"Connected to {self._e_host.text()}:{self._e_port.text()}")
        self._poll.start(200)

    def _on_conn_fail(self, msg: str):
        self._recv = None;  self._b_conn.setEnabled(True)
        self._log(f"Connection failed: {msg}")

    def _on_conn_lost(self):
        self._poll.stop();  self._recv = None
        self._b_conn.setText("Connect");  self._b_conn.setEnabled(True)
        self._e_host.setEnabled(True);  self._e_port.setEnabled(True)
        for b in self._mode_btns + [self._b_home, self._b_vs, self._b_save, self._b_log,
                                      self._b_set_target, self._b_fill_target]:
            b.setEnabled(False)
        self.statusBar().showMessage("Disconnected");  self._log("Disconnected")

    # ──────────────────────────────────────────────────────────────────────────
    # State display
    # ──────────────────────────────────────────────────────────────────────────
    def _query_state(self):
        self._send(CMD_STATE, SUBCMD_STATE_QUERY)

    def _on_state(self, st: dict):
        self._last_state = st
        self._l_mode.setText(_MODE_NAMES.get(st['mode'], f"?({st['mode']})"))

        vsn = _VS_NAMES.get(st['vs'], f"?({st['vs']})")
        self._l_vs.setText(vsn)
        self._l_vs.setStyleSheet(f"color:{_VS_COLORS.get(st['vs'], '#888')};")
        self._cam_vs_lbl.setText(f"VS: {vsn}")
        self._cam_vs_lbl.setStyleSheet(f"color:{_VS_COLORS.get(st['vs'], '#888')};")

        on_log = bool(st['logging'])
        self._l_log.setText("ON" if on_log else "OFF")
        self._l_log.setStyleSheet("color:#00cc44;" if on_log else "color:#888;")

        for k in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
            self._pose_lbls[k].setText(f"{st[k]:+.10f}")

        self._b_vs.blockSignals(True)
        self._b_vs.setChecked(st['vs'] != 0)
        self._b_vs.blockSignals(False)

        # Auto-manage camera when VS state changes
        prev = self._last_vs;  self._last_vs = st['vs']
        if prev == 0 and st['vs'] != 0:
            self._stop_camera()
            self._cam_label.setText(f"Visual Servo active\n({vsn})\n\nCamera released to C++")
        elif prev != 0 and st['vs'] == 0:
            QTimer.singleShot(800, self._start_camera)

    # ──────────────────────────────────────────────────────────────────────────
    # Commands
    # ──────────────────────────────────────────────────────────────────────────
    def _set_mode(self, code: int):
        self._send(CMD_CONTROL, SUBCMD_CTRL_SET_MODE, bytes([code]))
        self._log(f"→ Mode: {_MODE_NAMES.get(code, code)}")

    def _on_meta_robot(self, m: dict):
        self._meta_robot = m;  self._refresh_meta()

    def _on_meta_ecat(self, m: dict):
        self._meta_ecat = m;  self._refresh_meta()

    def _on_meta_axes(self, axes: list):
        self._meta_axes = axes;  self._refresh_meta()

    def _refresh_meta(self):
        lines = []
        if self._meta_robot:
            m = self._meta_robot
            lines += ['━━━ Robot ━━━',
                      f"  Name       : {m.get('name','?')}",
                      f"  Total Axes : {m.get('total_axes','?')}",
                      f"  Simulation : {'Yes' if m.get('is_sim') else 'No'}",
                      f"  EtherCAT   : {'Enabled' if m.get('ecat_enabled') else 'Disabled'}",
                      '']
        if self._meta_ecat:
            e = self._meta_ecat
            lines += ['━━━ EtherCAT ━━━',
                      f"  Master State : 0x{e.get('master_state',0):02X}",
                      f"  Slave State  : 0x{e.get('slave_state',0):02X}",
                      f"  Domain State : 0x{e.get('domain_state',0):02X}",
                      f"  Slave Count  : {e.get('slave_count','?')}",
                      '']
        if self._meta_axes:
            lines.append('━━━ Axes ━━━')
            for a in self._meta_axes:
                t = _AXIS_TYPES.get(a['type'], f"?({a['type']})")
                c = _COMM_TYPES.get(a['comm'], f"?({a['comm']})")
                u = a['pos_unit']
                lines += [f"  [{a['no']}] {a['name']}  ({t} / {c})",
                          f"      Pos : [{a['pos_l']:+.1f}, {a['pos_u']:+.1f}] {u}",
                          f"      Vel : [{a['vel_l']:+.3f}, {a['vel_u']:+.3f}] rad/s",
                          f"      Acc : [{a['acc_l']:+.3f}, {a['acc_u']:+.3f}] rad/s²",
                          '']
        self._meta_text.setPlainText('\n'.join(lines))

    def _go_home(self):
        self._send(CMD_CONTROL, SUBCMD_CTRL_HOME);  self._log("→ Home (all joints → 0, 5 s)")

    def _toggle_vs(self):
        self._send(CMD_CONTROL, SUBCMD_CTRL_VS_TOGGLE);  self._log("→ VS toggle")

    def _save_pose(self):
        self._send(CMD_CONTROL, SUBCMD_CTRL_SAVE_POSE);  self._log("→ Target pose saved")

    def _trigger_log(self):
        self._send(CMD_CONTROL, SUBCMD_CTRL_TRIGGER_LOG);  self._log("→ Log triggered (5 s)")

    def _send_target_pose(self):
        try:
            vals = [float(self._target_edits[k].text())
                    for k in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')]
        except ValueError:
            self._log("Target pose: invalid number");  return
        payload = struct.pack('<6d', *vals)
        self._send(CMD_CONTROL, SUBCMD_CTRL_SET_TARGET_POSE, payload)
        self._log(f"→ Target pose set: X={vals[0]:.4f} Y={vals[1]:.4f} Z={vals[2]:.4f} "
                  f"R={vals[3]:.4f} P={vals[4]:.4f} Yaw={vals[5]:.4f}")

    def _fill_target_from_tcp(self):
        for k in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
            txt = self._pose_lbls[k].text()
            if txt != '—':
                self._target_edits[k].setText(txt.strip())
        self._log("→ Target fields filled from current TCP pose")

    def _send(self, cmd: int, subcmd: int, data: bytes = b''):
        if self._recv:
            self._recv.send(_make_pkt(cmd, subcmd, data))

    # ──────────────────────────────────────────────────────────────────────────
    # Camera
    # ──────────────────────────────────────────────────────────────────────────
    def _start_camera(self):
        if self._cam and self._cam.isRunning(): return
        if not _RS:
            self._cam_label.setText("pyrealsense2 not installed\n\npip install pyrealsense2")
            return
        self._cam = _CamThread()
        self._cam.frame_ready.connect(self._on_frame)
        self._cam.cam_error.connect(self._on_cam_error)
        self._cam.finished.connect(self._on_cam_finished)
        self._cam.start()
        self._fps_timer.start()

    def _stop_camera(self):
        self._fps_timer.stop()
        if self._cam:
            self._cam.stop();  self._cam.wait(600)
            self._cam = None

    def _on_cam_error(self, msg: str):
        self._cam_label.setText(f"Camera error:\n{msg}")
        self._log(f"Cam error: {msg}")

    def _on_cam_finished(self):
        self._fps_timer.stop()

    def _on_frame(self, frame: np.ndarray):
        self._cam_frames += 1
        h, w = frame.shape[:2]
        rgb = np.ascontiguousarray(frame[:, :, ::-1])
        qi  = QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888).copy()
        px  = QPixmap.fromImage(qi).scaled(
            self._cam_label.width(), self._cam_label.height(),
            Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self._cam_label.setPixmap(px)

    def _update_fps(self):
        fps = self._cam_frames;  self._cam_frames = 0
        if fps > 0: self._cam_fps_lbl.setText(f"{fps} fps")

    # ──────────────────────────────────────────────────────────────────────────
    # Plots
    # ──────────────────────────────────────────────────────────────────────────
    def _browse_traj(self):
        paths, _ = QFileDialog.getOpenFileNames(
            self, "Select DataLog CSV files", _RT_LOG_DIR,
            "CSV Files (*.csv);;All (*)")
        if paths:
            self._traj_paths = paths
            names = [os.path.basename(p) for p in paths]
            self._l_traj_files.setText(f"{len(paths)} file(s): {', '.join(names)}")
            self._b_traj_plot.setEnabled(True)

    def _do_traj_plot(self):
        if not self._traj_paths: return
        try:
            fig1, fig2 = _plot_tcp(self._traj_paths)
            self._traj_figs = [fig1, fig2]
            self._replace_canvases(self._traj_vlay, [fig1, fig2])
            self._b_traj_save.setEnabled(True)
            self._log("→ TCP trajectory plotted")
        except Exception as e:
            self._log(f"Plot error: {e}")

    def _replace_canvases(self, layout: QVBoxLayout, figs: list):
        """Clear layout and insert new FigureCanvas widgets for each figure."""
        while layout.count():
            item = layout.takeAt(0)
            if item.widget(): item.widget().setParent(None)
        for fig in figs:
            canvas = FigureCanvas(fig)
            h_px   = int(fig.get_figheight() * fig.dpi)
            canvas.setMinimumHeight(h_px)
            canvas.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            layout.addWidget(canvas)
        layout.addStretch()

    def _save_png(self, which: str):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_dir = os.path.join(_RT_LOG_DIR, "plots")
        os.makedirs(out_dir, exist_ok=True)
        for i, fig in enumerate(self._traj_figs):
            path = os.path.join(out_dir, f"ui_log{i+1}_{stamp}.png")
            fig.savefig(path, dpi=150);  self._log(f"Saved: {path}")

    # ──────────────────────────────────────────────────────────────────────────
    # Misc
    # ──────────────────────────────────────────────────────────────────────────
    def _log(self, msg: str):
        ts = datetime.now().strftime("%H:%M:%S")
        self._log_w.append(f"[{ts}] {msg}")

    def closeEvent(self, event):
        self._stop_camera()
        if self._recv: self._recv.stop();  self._recv.wait(1000)
        event.accept()


# ═══════════════════════════════════════════════════════════════════════════════
if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())
