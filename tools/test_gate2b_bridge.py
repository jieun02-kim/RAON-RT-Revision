#!/usr/bin/env python3
"""Gate 2b scripted verification — camera-free, robot-free.

Pieces under test (all real):
  - CROS2PickBridge (C++), via the gate2b_bridge_test.out harness
  - the real pick_logic node's desired_class LIVE parameter service

Synthetic pieces (this driver):
  - /detections feeder      (apple + banana, 15 Hz)
  - /pick_target_base feeder (valid apple @ base (0.45, 0.10, 0.12) ± 1.5 mm)

Scripted flow == operator flow:  p → menu → '1'(apple) → param set →
TARGET LOCKED → v → stats gate → GOAL READY → harness consumes goal.

Checks:
  C1 menu lists apple/banana (+AUTO)
  C2 bridge reports desired_class='apple' set
  C3 pick_logic REALLY has desired_class=apple   (ros2 param get)
  C4 TARGET LOCKED: apple seen
  C5 goal consumed: class=apple, z == 0.12 + 0.15 margin (±0.02)

Run:  source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
      python3 ~/RAON-RT-Revision/tools/test_gate2b_bridge.py

Hardening carried over from ros2_ws/tools/vision/test_pick_logic_v2.py:
stale-node preflight with /proc comm check, ros2 daemon reset, process-group
spawn + unconditional SIGKILL teardown, retrying ros2 CLI calls.
"""
import os
import random
import re
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from my_interfaces.msg import Detection, DetectionArray, PickTarget3D

HARNESS_BIN = os.path.expanduser(
    '~/RAON-RT-Revision/App/Indy7/bin/gate2b_bridge_test.out')

BASE_XYZ = (0.45, 0.10, 0.12)   # feeder ground truth (base_link)
Z_MARGIN = 0.15                  # bridge default — goal z must be 0.27


def _kill_stale():
    """pick_logic / harness leftovers poison discovery — refuse to test dirty."""
    pats = ['install/pick_logic_pkg', 'gate2b_bridge_test.out']
    killed = False
    for pat in pats:
        pids = subprocess.run(['pgrep', '-f', pat],
                              capture_output=True, text=True).stdout.split()
        for pid in pids:
            try:
                with open(f'/proc/{pid}/comm') as f:
                    comm = f.read().strip()
            except OSError:
                continue
            if comm not in ('python3', 'pick_logic', 'gate2b_bridge_t',
                            'gate2b_bridge_test.out'):
                continue
            print(f'preflight: killing stale pid {pid} ({comm})')
            subprocess.run(['kill', '-9', pid], capture_output=True)
            killed = True
    if killed:
        time.sleep(1.0)
    subprocess.run(['ros2', 'daemon', 'stop'], capture_output=True)
    time.sleep(0.5)


class Feeder(Node):
    def __init__(self):
        super().__init__('gate2b_feeder')
        self.pub_det = self.create_publisher(DetectionArray, '/detections', 10)
        self.pub_tgt = self.create_publisher(PickTarget3D, '/pick_target_base', 10)
        self.rng = random.Random(42)

    def _det(self, cls, conf, cx, cy):
        d = Detection()
        d.class_id = 0
        d.class_name = cls
        d.confidence = float(conf)
        d.center_x, d.center_y = float(cx), float(cy)
        d.width, d.height = 80.0, 80.0
        return d

    def tick(self):
        now = self.get_clock().now().to_msg()
        arr = DetectionArray()
        arr.header.stamp = now
        arr.detections = [self._det('apple', 0.95, 424, 240),
                          self._det('banana', 0.70, 600, 200)]
        self.pub_det.publish(arr)

        t = PickTarget3D()
        t.header.stamp = now
        t.header.frame_id = 'base_link'
        t.capture_stamp = now
        t.target_valid = True
        t.depth_valid = True
        t.class_id = 0
        t.class_name = 'apple'
        t.confidence = 0.95
        t.center_x, t.center_y = 424.0, 240.0
        t.width, t.height = 80.0, 80.0
        t.x = BASE_XYZ[0] + self.rng.gauss(0.0, 0.0015)
        t.y = BASE_XYZ[1] + self.rng.gauss(0.0, 0.0015)
        t.z = BASE_XYZ[2] + self.rng.gauss(0.0, 0.0015)
        self.pub_tgt.publish(t)


def ros2_cli(args, tries=5, delay=1.0):
    last = None
    for _ in range(tries):
        r = subprocess.run(['ros2'] + args, capture_output=True, text=True)
        if r.returncode == 0:
            return r.stdout
        last = r
        time.sleep(delay)
    raise RuntimeError(f'ros2 {" ".join(args)} failed: '
                       f'{(last.stderr or last.stdout).strip()}')


def killpg_hard(proc):
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        return
    try:
        os.killpg(pgid, signal.SIGTERM)
        proc.wait(timeout=2)
    except (subprocess.TimeoutExpired, ProcessLookupError):
        pass
    try:
        os.killpg(pgid, signal.SIGKILL)
    except ProcessLookupError:
        pass


def main():
    if not os.path.exists(HARNESS_BIN):
        print(f'FATAL: harness missing — build first: '
              f'cd ~/RAON-RT-Revision/App/Indy7 && make gate2b_test')
        return 1

    _kill_stale()
    rclpy.init()
    feeder = Feeder()

    stop_feed = threading.Event()

    def feed_loop():
        while not stop_feed.is_set():
            feeder.tick()
            time.sleep(1.0 / 15.0)

    th_feed = threading.Thread(target=feed_loop, daemon=True)
    th_feed.start()

    pick_logic = subprocess.Popen(
        ['ros2', 'run', 'pick_logic_pkg', 'pick_logic', '--ros-args',
         '-p', 'log_period_sec:=999.0'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True)

    harness = subprocess.Popen(
        [HARNESS_BIN], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True, bufsize=1,
        start_new_session=True)

    lines = []
    lines_lock = threading.Lock()

    def reader():
        for line in harness.stdout:
            with lines_lock:
                lines.append(line.rstrip())
            print('  |', line.rstrip())

    th_read = threading.Thread(target=reader, daemon=True)
    th_read.start()

    def wait_for(substr, timeout):
        t_end = time.time() + timeout
        seen = 0
        while time.time() < t_end:
            with lines_lock:
                for i in range(seen, len(lines)):
                    if substr in lines[i]:
                        return lines[i]
                seen = len(lines)
            time.sleep(0.1)
        return None

    def send(cmd):
        harness.stdin.write(cmd + '\n')
        harness.stdin.flush()

    passed, failed = [], []

    def check(name, cond, detail):
        (passed if cond else failed).append(name)
        print(f"[{'PASS' if cond else 'FAIL'}] {name}: {detail}")

    try:
        if wait_for('[PickBridge] up', 15) is None:
            print('FATAL: bridge never came up')
            return 1
        time.sleep(2.5)          # let the feeder fill the menu window

        # C1 — menu
        send('p')
        l_apple = wait_for('1) apple', 5)
        l_banana = wait_for('2) banana', 2)
        l_auto = wait_for('3) AUTO', 2)
        check('C1 menu', all([l_apple, l_banana, l_auto]),
              f'apple={bool(l_apple)} banana={bool(l_banana)} auto={bool(l_auto)}')

        # C2 — select apple → param set through the bridge
        send('1')
        l_set = wait_for("desired_class='apple' set", 10)
        check('C2 param-set ack', l_set is not None, l_set or 'no ack line')

        # C3 — pick_logic really has it
        try:
            out = ros2_cli(['param', 'get', '/pick_logic_node', 'desired_class'])
            check('C3 param on node', 'apple' in out, out.strip())
        except RuntimeError as e:
            check('C3 param on node', False, str(e))

        # C4 — locked (auto-lock from feeder start also satisfies this)
        l_lock = wait_for('TARGET LOCKED: apple', 5)
        check('C4 target locked', l_lock is not None, l_lock or 'no lock line')

        # C5 — approach: stats gate → goal → harness consumption
        send('v')
        l_goal = wait_for('[HARNESS] GOAL CONSUMED: class=apple', 10)
        ok, detail = False, l_goal or 'no goal line'
        if l_goal:
            m = re.search(r'xyz=\(([-\d.]+), ([-\d.]+), ([-\d.]+)\)', l_goal)
            if m:
                gx, gy, gz = (float(m.group(i)) for i in (1, 2, 3))
                ok = (abs(gx - BASE_XYZ[0]) < 0.02 and
                      abs(gy - BASE_XYZ[1]) < 0.02 and
                      abs(gz - (BASE_XYZ[2] + Z_MARGIN)) < 0.02)
                detail = f'goal=({gx:.3f}, {gy:.3f}, {gz:.3f}), ' \
                         f'expect z≈{BASE_XYZ[2] + Z_MARGIN:.3f}'
        check('C5 gated goal', ok, detail)

        send('q')
        try:
            harness.wait(timeout=5)
        except subprocess.TimeoutExpired:
            pass

    finally:
        stop_feed.set()
        killpg_hard(harness)
        killpg_hard(pick_logic)
        feeder.destroy_node()
        rclpy.shutdown()

    print(f'\n==> {len(passed)} PASS / {len(failed)} FAIL'
          + (f'  (failed: {failed})' if failed else ''))
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
