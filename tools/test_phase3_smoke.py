#!/usr/bin/env python3
"""Phase 3 — servo-off integration run (motion 0), scripted.

What runs (all real):
  - perception pipeline (ros2 launch system_bringup_pkg pick_place_vitis_ai.launch.py)
  - Indy7Ctrl.out on the live EtherCAT bus (7 slaves → OP), servo-off by cfg
    (SIM=0, AUTO_SERVO_ON=0 all axes, ENABLE_CONTROLLER_AT_STARTUP=0 — the
    controller is never enabled here, so a bridge goal can never be consumed)

Checks:
  P0 preflight  rtprio>=97, 7 slaves answering, binary/launch present, clean host
  P1 pipeline   /detections functionally flowing (first DetectionArray received)
  P2 app up     [PickBridge] up + CRobot init + ALL 7 slaves reach OP
  P3 OP hold    60 s: app alive, slaves stay OP, Lost frames delta 0, no ERROR
  P4 bridge     in-app 'p' menu renders from the LIVE pipeline (+ select+'v'
                path when a pickable class is actually in view)
  P5 joints     per-axis 0x6064 SDO read during OP (baseline for the manual
                hand-move readout check afterwards)
  P6 shutdown   SIGINT → CRobot::DeInit path (pos save, master release,
                [PickBridge] down, APPLICATION ENDED) → slaves back to PREOP

Run:  source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
      python3 ~/RAON-RT-Revision/tools/test_phase3_smoke.py [--app-only]

--app-only: skip the pipeline and the bridge menu interaction — validates the
app+bus lifecycle (OP hold, SDO, SIGINT teardown) in isolation first. Used to
stage the rerun after the 2026-07-26 board freeze.

Logs: ~/RAON-RT-Revision/tools/phase3_logs/{pipeline.log,app.log}
"""
import os
import re
import resource
import signal
import subprocess
import sys
import threading
import time

APP_DIR = os.path.expanduser('~/RAON-RT-Revision/App/Indy7')
APP_BIN = './bin/Indy7Ctrl.out'
LOG_DIR = os.path.expanduser('~/RAON-RT-Revision/tools/phase3_logs')
OP_HOLD_S = 60   # override: --hold <seconds>
if '--hold' in sys.argv:
    OP_HOLD_S = int(sys.argv[sys.argv.index('--hold') + 1])


def sh(cmd):
    return subprocess.run(cmd, capture_output=True, text=True).stdout


def slave_states():
    out = sh(['ethercat', 'slaves'])
    states = re.findall(r'^\s*\d+\s+\d+:\d+\s+(\S+)', out, re.M)
    return states


def lost_frames():
    m = re.findall(r'Lost frames:\s+(\d+)', sh(['ethercat', 'master']))
    return max(int(x) for x in m) if m else -1


def find_claude_ancestor():
    p = os.getpid()
    while p != 1:
        try:
            with open(f'/proc/{p}/comm') as f:
                comm = f.read().strip()
            with open(f'/proc/{p}/stat') as f:
                ppid = int(f.read().split(')')[1].split()[1])
        except OSError:
            break
        if comm == 'claude':
            return p
        p = ppid
    return None


def killpg_hard(proc, sig=signal.SIGTERM, wait=5):
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        return
    try:
        os.killpg(pgid, sig)
        proc.wait(timeout=wait)
    except (subprocess.TimeoutExpired, ProcessLookupError):
        pass
    try:
        os.killpg(pgid, signal.SIGKILL)
    except ProcessLookupError:
        pass


def main():
    os.makedirs(LOG_DIR, exist_ok=True)
    passed, failed, notes = [], [], []

    def check(name, cond, detail):
        (passed if cond else failed).append(name)
        print(f"[{'PASS' if cond else 'FAIL'}] {name}: {detail}")

    app_only = '--app-only' in sys.argv

    # ---------------- P0 preflight ----------------
    rt = resource.getrlimit(resource.RLIMIT_RTPRIO)[0]
    if rt < 97:
        anc = find_claude_ancestor()
        hint = (f'sudo prlimit --rtprio=98:98 --pid {anc}'
                if anc else 'run from a fresh login shell (limits.d applies)')
        print(f'FATAL: RLIMIT_RTPRIO={rt} < 97 — RT tasks cannot start.\n'
              f'  unblock: {hint}   (then rerun this script)')
        return 1
    # The app runs mlockall(MCL_CURRENT|MCL_FUTURE): with a finite memlock cap
    # the DDS/ROS2 threads' malloc arenas blow past it → std::bad_alloc mid-OP
    # (observed 2026-07-26, followed by a board freeze during the teardown).
    ml = resource.getrlimit(resource.RLIMIT_MEMLOCK)[0]
    if ml != resource.RLIM_INFINITY:
        anc = find_claude_ancestor()
        hint = (f'sudo prlimit --memlock=unlimited:unlimited --pid {anc}'
                if anc else 'run from a fresh login shell (limits.d applies)')
        print(f'FATAL: RLIMIT_MEMLOCK={ml} (need unlimited — mlockall app).\n'
              f'  unblock: {hint}   (then rerun this script)')
        return 1
    st = slave_states()
    if len(st) != 7:
        print(f'FATAL: expected 7 slaves, got {len(st)} — robot power / master?')
        return 1
    if not os.path.exists(os.path.join(APP_DIR, APP_BIN)):
        print('FATAL: app binary missing — cd App/Indy7 && make')
        return 1
    for pat, why in [('realsense2_camera', 'pipeline already running'),
                     ('Indy7Ctrl.out', 'app already running')]:
        if subprocess.run(['pgrep', '-f', pat], capture_output=True).returncode == 0:
            print(f'FATAL: {why} — stop it first (avoid double camera/bus owner)')
            return 1
    subprocess.run(['ros2', 'daemon', 'stop'], capture_output=True)
    print(f'[P0] preflight ok: rtprio={rt}, slaves={st}')

    lost0 = lost_frames()

    # ---------------- P1 pipeline ----------------
    pipeline = None
    probe = None
    rclpy = None
    if app_only:
        print('[P1] SKIPPED (--app-only)')
    else:
        pipe_log = open(os.path.join(LOG_DIR, 'pipeline.log'), 'w')
        pipeline = subprocess.Popen(
            ['ros2', 'launch', 'system_bringup_pkg', 'pick_place_vitis_ai.launch.py'],
            stdout=pipe_log, stderr=subprocess.STDOUT, start_new_session=True)

        import rclpy
        from rclpy.node import Node as RclNode
        from my_interfaces.msg import DetectionArray
        rclpy.init()
        probe = RclNode('phase3_probe')
        got = []
        probe.create_subscription(DetectionArray, '/detections',
                                  lambda m: got.append(len(m.detections)), 10)
        t_end = time.time() + 60
        while time.time() < t_end and not got:
            rclpy.spin_once(probe, timeout_sec=0.5)
        check('P1 pipeline', bool(got),
              f'first /detections after launch (dets={got[0] if got else "-"})')
    app = None
    try:
        if not app_only and not got:
            raise RuntimeError('pipeline dead — aborting the run')

        # ---------------- P2 app + OP ----------------
        app_log_f = open(os.path.join(LOG_DIR, 'app.log'), 'w')
        app_env = dict(os.environ)
        # mlockall(MCL_FUTURE) locks every malloc arena the ROS2/DDS threads
        # create (64 MB virtual each by default) — cap arenas to keep the
        # locked footprint sane. stdbuf -oL: keep app stdout line-buffered
        # through the pipe so a crash cannot swallow the log tail again.
        app_env['MALLOC_ARENA_MAX'] = '2'
        app = subprocess.Popen(
            ['stdbuf', '-oL', APP_BIN, '-f', 'INDY7.cfg'], cwd=APP_DIR,
            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, bufsize=1,
            env=app_env, start_new_session=True)
        lines = []
        llock = threading.Lock()

        def reader():
            for ln in app.stdout:
                with llock:
                    lines.append(ln.rstrip())
                app_log_f.write(ln)
                app_log_f.flush()
        threading.Thread(target=reader, daemon=True).start()

        def wait_line(substr, timeout):
            t_e = time.time() + timeout
            seen = 0
            while time.time() < t_e:
                with llock:
                    for i in range(seen, len(lines)):
                        if substr in lines[i]:
                            return lines[i]
                    seen = len(lines)
                if app.poll() is not None:
                    return None
                time.sleep(0.2)
            return None

        bridge_up = wait_line('[PickBridge] up', 20)
        t_e = time.time() + 40
        all_op = False
        while time.time() < t_e and not all_op:
            s = slave_states()
            all_op = (len(s) == 7 and all(x == 'OP' for x in s))
            if app.poll() is not None:
                break
            time.sleep(1.0)
        check('P2 app+OP', bool(bridge_up) and all_op,
              f'bridge_up={bool(bridge_up)}, slaves={slave_states()}')
        if app.poll() is not None:
            raise RuntimeError('app died during init')

        # ---------------- P3 OP hold ----------------
        # Re-baseline the lost-frame counter AFTER OP: the PREOP→OP activation
        # itself normally drops a couple frames (state transitions + SDO burst)
        # — steady-state loss is what the gate is about.
        time.sleep(2.0)
        lost0 = lost_frames()
        with llock:
            n_before = len(lines)
        t_e = time.time() + OP_HOLD_S
        mid_ok = True
        while time.time() < t_e:
            time.sleep(5.0)
            if app.poll() is not None or \
               not all(x == 'OP' for x in slave_states()):
                mid_ok = False
                break
        lost1 = lost_frames()
        with llock:
            new_err = [l for l in lines[n_before:] if 'ERROR' in l]
        # IgH 'Lost frames' is a derived stat that can DECREASE when a late
        # frame is eventually accounted — only an INCREASE means real loss.
        check('P3 OP-hold', mid_ok and lost1 <= lost0 and not new_err,
              f'{OP_HOLD_S}s: alive={app.poll() is None}, '
              f'lost {lost0}→{lost1}, errors={new_err or "none"}')

        # ---------------- P4 bridge in-app ----------------
        if app_only:
            print('[P4] SKIPPED (--app-only, no pipeline)')
        else:
            app.stdin.write('p')
            app.stdin.flush()
            menu = wait_line('---- visible objects', 10)
            time.sleep(1.0)
            with llock:
                entries = {}
                for l in lines[-25:]:
                    m = re.search(r'\[PickBridge\]\s+(\d)\)\s+(\w+)\s+\(', l)
                    if m:
                        entries[m.group(2)] = m.group(1)
            p4_detail = f'menu={bool(menu)}, classes={list(entries) or "(none in view)"}'
            p4_ok = bool(menu)
            if entries:
                cls, digit = next(iter(entries.items()))
                app.stdin.write(digit)
                app.stdin.flush()
                set_ln = wait_line(f"desired_class='{cls}' set", 15)
                app.stdin.write('v')
                app.stdin.flush()
                gate_ln = (wait_line('GOAL READY', 15) or
                           wait_line('GATE FAIL', 1) or
                           wait_line('no live valid target', 1))
                p4_ok = p4_ok and bool(set_ln) and bool(gate_ln)
                p4_detail += f' | select {cls}: set={bool(set_ln)}, gate="{gate_ln}"'
                notes.append('controller disabled → a GOAL READY stays unconsumed '
                             '(RT guard) — by design in Phase 3')
            else:
                notes.append("menu empty — put a pickable object in view to "
                             "exercise select+'v' in-app (path proven in Gate 2b)")
            check('P4 bridge', p4_ok, p4_detail)

        # ---------------- P5 joint SDO ----------------
        vals = []
        for ax in range(6):
            out = sh(['ethercat', 'upload', f'-p{ax}', '0x6064', '0']).split()
            vals.append(int(out[1]) if len(out) > 1 else None)
        check('P5 joint-SDO', all(v is not None for v in vals),
              f'0x6064 counts={vals}')
        notes.append('hand-move check: move a joint by hand, rerun the SDO '
                     'read, counts must follow (servo-off = freely movable)')

        # ---------------- P6 shutdown ----------------
        os.kill(app.pid, signal.SIGINT)
        ended = wait_line('APPLICATION ENDED', 25)
        try:
            app.wait(timeout=10)
        except subprocess.TimeoutExpired:
            pass
        t_e = time.time() + 15
        preop = False
        while time.time() < t_e and not preop:
            s = slave_states()
            preop = (len(s) == 7 and all(x in ('PREOP', 'SAFEOP') for x in s))
            time.sleep(1.0)
        with llock:
            deinit = any('(DeInit)' in l for l in lines)
            bdown = any('[PickBridge] down' in l for l in lines)
        check('P6 shutdown', bool(ended) and app.poll() is not None and
              preop and deinit and bdown,
              f'ended={bool(ended)}, rc={app.poll()}, slaves={slave_states()}, '
              f'deinit_log={deinit}, bridge_down={bdown}')

    except RuntimeError as e:
        print(f'ABORT: {e}')
        failed.append(f'ABORT({e})')
    finally:
        if app is not None:
            killpg_hard(app, signal.SIGKILL, 2)
        if pipeline is not None:
            killpg_hard(pipeline, signal.SIGINT, 12)
            pipe_log.close()
        if probe is not None:
            probe.destroy_node()
            rclpy.shutdown()

    print('\n-- notes --')
    for n in notes:
        print(' *', n)
    print(f'\n==> {len(passed)} PASS / {len(failed)} FAIL'
          + (f'  (failed: {failed})' if failed else ''))
    print(f'logs: {LOG_DIR}/')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
