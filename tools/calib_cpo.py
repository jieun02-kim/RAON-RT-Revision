#!/usr/bin/env python3
"""Phase 5-2: image%04d.png -> pose_cPo_N.yaml (camera->tag pose).

Detects the flange AprilTag (36h11) in every captured image and solves the
planar pose with SOLVEPNP_IPPE_SQUARE (well-conditioned for one square;
reports the two-solution ambiguity ratio so flip-prone frames are visible).
Output is vpPoseVector yaml (tx ty tz + theta-u), the exact format
App/CalibUtils/eye_to_hand_calib.py already reads — run that afterwards
from the same directory.

Intrinsics come from camera_info.yaml saved by calib_grab.py (factory
calibration = same camera model the perception pipeline's 3D uses, D12).

Usage: python3 calib_cpo.py --tag-mm 65.0
       (--dir defaults to App/CalibUtils/kv260)
"""
import argparse
import glob
import os
import re
import sys

import cv2
import numpy as np
import yaml


def load_intrinsics(path):
    with open(path) as f:
        info = yaml.safe_load(f)
    K = np.array(info['K'], dtype=np.float64).reshape(3, 3)
    D = np.array(info['D'], dtype=np.float64)
    return K, D


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tag-mm', type=float, required=True,
                    help='black square outer side length [mm]')
    ap.add_argument('--dir', default=os.path.expanduser(
        '~/RAON-RT-Revision/App/CalibUtils/kv260'))
    args = ap.parse_args()

    s = args.tag_mm / 1000.0
    # Corner order must match detectMarkers: TL, TR, BR, BL (marker frame:
    # x right, y up, z out of the tag face) — same as estimatePoseSingleMarkers.
    obj = np.array([[-s / 2, s / 2, 0], [s / 2, s / 2, 0],
                    [s / 2, -s / 2, 0], [-s / 2, -s / 2, 0]], dtype=np.float64)

    K, D = load_intrinsics(os.path.join(args.dir, 'camera_info.yaml'))
    print(f'[cpo] tag side {args.tag_mm:.1f} mm, fx={K[0,0]:.1f} fy={K[1,1]:.1f}')

    adict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    # Best corners first, but fall back — CORNER_REFINE_APRILTAG can DROP a
    # marker the default detector accepts (seen on image0012). A missing index
    # would silently misalign eye_to_hand_calib.py's sorted-zip pairing, so a
    # coarser detection beats no detection.
    param_sets = []
    for method in ('CORNER_REFINE_APRILTAG', 'CORNER_REFINE_SUBPIX', None):
        p = cv2.aruco.DetectorParameters_create()
        if method is not None:
            try:
                p.cornerRefinementMethod = getattr(cv2.aruco, method)
            except AttributeError:
                continue
        param_sets.append((method or 'default', p))

    def detect(img):
        for name, p in param_sets:
            corners, ids, _ = cv2.aruco.detectMarkers(img, adict, parameters=p)
            if ids is not None and len(ids) > 0:
                return corners, name
        return None, None

    imgs = sorted(glob.glob(os.path.join(args.dir, 'image*.png')))
    if not imgs:
        print('[cpo] no images found'); return 1

    n_ok = 0
    for p in imgs:
        m = re.search(r'image(\d+)\.png$', p)
        idx = int(m.group(1))
        img = cv2.imread(p)
        corners, used = detect(img)
        if corners is None:
            print(f'image{idx:04d}: NO TAG — skipped (pose pair {idx} unusable)')
            continue
        pts = corners[0][0].astype(np.float64)

        ok, rvecs, tvecs, errs = cv2.solvePnPGeneric(
            obj, pts, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            print(f'image{idx:04d}: solvePnP FAILED'); continue
        errs = np.asarray(errs).flatten()
        best = int(np.argmin(errs))
        rvec, tvec = rvecs[best].flatten(), tvecs[best].flatten()
        ratio = (errs[1 - best] / max(errs[best], 1e-9)) if len(errs) > 1 else np.inf
        flag = '  AMBIGUOUS!' if ratio < 2.0 else ''

        out = os.path.join(args.dir, f'pose_cPo_{idx}.yaml')
        with open(out, 'w') as f:
            f.write('rows: 6\ncols: 1\ndata: \n')
            for v in list(tvec) + list(rvec):
                f.write(f'  - [{v:.9f}]\n')
        n_ok += 1
        note = '' if used == 'CORNER_REFINE_APRILTAG' else f'  [{used}]'
        print(f'image{idx:04d}: z={tvec[2]:.3f} m  reproj={errs[best]:.2f} px  '
              f'alt/best={ratio:5.1f}{flag}{note}')

    print(f'[cpo] {n_ok}/{len(imgs)} pose_cPo written to {args.dir}')
    return 0 if n_ok == len(imgs) else 1


if __name__ == '__main__':
    sys.exit(main())
