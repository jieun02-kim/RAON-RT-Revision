#!/usr/bin/env python3
"""Phase 5 hand-eye calibration — image grabber (D11: pipeline topics only).

Run WHILE the perception pipeline is up (owns the camera) and the robot app
is in grav-comp ('h'). Per pose the operator:

  1. hand-guides the arm so the flange AprilTag faces the camera
  2. presses Enter HERE  → latest frame is tag-checked; saved only when the
     tag is actually detected (image%04d.png)
  3. presses 's' in the app → pose_rPe_%d.yaml (SAME index — the app's
     CalibCapture counter and this one must stay in lockstep, which is why
     the image is confirmed FIRST)

camera_info is saved once (camera_info.yaml) — factory intrinsics, the same
model the pipeline's 3D uses (D12).

Run:  source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
      python3 ~/RAON-RT-Revision/tools/calib_grab.py

Output: ~/RAON-RT-Revision/App/CalibUtils/kv260/
"""
import os
import sys
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, CameraInfo

OUT_DIR = os.path.expanduser('~/RAON-RT-Revision/App/CalibUtils/kv260')
TOPIC_IMG = '/camera/camera/color/image_raw/compressed'
TOPIC_INFO = '/camera/camera/color/camera_info'


class Grabber(Node):
    def __init__(self):
        super().__init__('calib_grab')
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.lock = threading.Lock()
        self.latest = None          # (bytes, stamp)
        self.info = None
        self.create_subscription(CompressedImage, TOPIC_IMG, self._on_img, qos)
        self.create_subscription(CameraInfo, TOPIC_INFO, self._on_info, qos)
        self.detector = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_APRILTAG_36h11)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

    def _on_img(self, msg):
        with self.lock:
            self.latest = bytes(msg.data)

    def _on_info(self, msg):
        if self.info is None:
            self.info = msg

    def snap(self):
        with self.lock:
            buf = self.latest
        if buf is None:
            return None, 'no frame yet'
        img = cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return None, 'decode failed'
        corners, ids, _ = cv2.aruco.detectMarkers(
            img, self.detector, parameters=self.aruco_params)
        if ids is None or len(ids) == 0:
            return None, 'TAG NOT FOUND'
        c = corners[0][0]
        side = float(np.linalg.norm(c[0] - c[1]))
        return img, f'tag id={int(ids[0][0])}, side≈{side:.0f}px'


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    rclpy.init()
    node = Grabber()
    th = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    th.start()

    print(f'[grab] waiting for camera_info + first frame on {TOPIC_IMG} ...')
    import time
    t0 = time.time()
    while (node.info is None or node.latest is None) and time.time() - t0 < 30:
        time.sleep(0.2)
    if node.info is None or node.latest is None:
        print('[grab] FATAL: no camera data — is the pipeline running?')
        return 1

    info = node.info
    with open(os.path.join(OUT_DIR, 'camera_info.yaml'), 'w') as f:
        f.write(f'width: {info.width}\nheight: {info.height}\n')
        f.write(f'distortion_model: {info.distortion_model}\n')
        f.write('K: [' + ', '.join(f'{v:.6f}' for v in info.k) + ']\n')
        f.write('D: [' + ', '.join(f'{v:.6f}' for v in info.d) + ']\n')
    print(f'[grab] camera_info saved ({info.width}x{info.height}, '
          f'fx={info.k[0]:.1f} fy={info.k[4]:.1f})')
    print('[grab] 순서: 자세 잡기 → 여기서 Enter(태그 확인+저장) → 앱에서 \'s\'')
    print('[grab] Enter=캡처, q+Enter=종료\n')

    n = 0
    while True:
        try:
            line = input(f'#{n + 1} 대기 — Enter: ')
        except EOFError:
            break
        if line.strip().lower() == 'q':
            break
        img, detail = node.snap()
        if img is None:
            print(f'   ✗ {detail} — 저장 안 됨, 자세 조정 후 다시 Enter')
            continue
        n += 1
        path = os.path.join(OUT_DIR, f'image{n:04d}.png')
        cv2.imwrite(path, img)
        print(f'   ✓ SAVED image{n:04d}.png ({detail})')
        print(f'   → 이제 앱에서 \'s\' 누르세요 (pose_rPe_{n}.yaml 이 되어야 함)')

    print(f'\n[grab] 총 {n}쌍 캡처. 앱 pose_rPe 카운트와 일치하는지 확인하세요.')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
