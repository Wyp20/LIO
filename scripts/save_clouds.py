#!/usr/bin/env python3
"""Subscribe to an algo cloud topic and save dense ascii XYZ (no downsample).

Uses a background writer thread so ROS callbacks stay fast enough to keep up
with offline publishers (together with LIO_CLOUD_PACE_MS on the publisher side).

Default output root: /media/wyp/娱乐/lio_eval_clouds/
"""
from __future__ import annotations

import argparse
import json
import queue
import signal
import sys
import threading
import time
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

DEFAULT_TOPICS = {
    "faster_lio": "/cloud_registered",
    "fast_lio": "/cloud_registered",
    "point_lio": "/cloud_registered",
    "pv_lio": "/cloud_registered",
    "akf_lio": "/cloud_reg_world",
    "voxel_map_plus": "/cloud_registered_surf",
    "rvoxelmap": "/rvoxelmap/cloud_registered",
    "super_lio": "/lio/cloud_world",
    "dlio": "/robot/dlio/odom_node/pointcloud/deskewed",
    "bievr": "/bievr_lio/points/registered",
}


def voxel_downsample(xyz: np.ndarray, leaf: float) -> np.ndarray:
    """leaf<=0 means no downsample (return input)."""
    if xyz.size == 0 or leaf <= 0:
        return xyz
    keys = np.floor(xyz / leaf).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return xyz[np.sort(idx)]


def write_xyz_txt(path: Path, xyz: np.ndarray, append: bool = False) -> None:
    mode = "ab" if append else "wb"
    # Large buffer + vectorized float→bytes beats np.savetxt for dense dumps.
    with path.open(mode, buffering=8 * 1024 * 1024) as f:
        # "%.4f" is enough for map viz; ~30% less I/O than %.6f
        np.savetxt(f, xyz, fmt="%.4f")


class CloudRecorder:
    def __init__(self, leaf: float, every_n: int, merge_path: Optional[Path]):
        self.leaf = leaf
        self.every_n = max(1, every_n)
        self.merge_path = merge_path
        self.lock = threading.Lock()
        self.frames_meta = []  # type: List[Tuple[float, int]]
        self.n_raw = 0
        self.n_kept = 0
        self.n_pts_raw = 0
        self.n_pts_saved = 0
        self.n_dropped = 0
        self.q = queue.Queue(maxsize=4000)  # type: queue.Queue
        self._stop = threading.Event()
        if self.merge_path is not None and self.merge_path.exists():
            self.merge_path.unlink()
        self.writer = threading.Thread(target=self._writer_loop, daemon=True)
        self.writer.start()

    def _writer_loop(self) -> None:
        while not self._stop.is_set() or not self.q.empty():
            try:
                item = self.q.get(timeout=0.2)
            except queue.Empty:
                continue
            if item is None:
                self.q.task_done()
                break
            stamp, xyz = item
            if self.merge_path is not None:
                write_xyz_txt(self.merge_path, xyz, append=True)
            with self.lock:
                self.n_pts_saved += xyz.shape[0]
                self.n_kept += 1
                self.frames_meta.append((stamp, int(xyz.shape[0])))
            self.q.task_done()

    def cb(self, msg: PointCloud2) -> None:
        self.n_raw += 1
        if (self.n_raw - 1) % self.every_n != 0:
            return
        pts = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if not pts:
            return
        xyz = np.asarray(pts, dtype=np.float64)
        self.n_pts_raw += xyz.shape[0]
        xyz_out = voxel_downsample(xyz, self.leaf)
        stamp = msg.header.stamp.to_sec() if msg.header.stamp else time.time()
        try:
            self.q.put_nowait((stamp, xyz_out))
        except queue.Full:
            self.n_dropped += 1

    def finish(self) -> None:
        # Block until sentinel is accepted; never time out — dense USB dumps
        # can take >>10min and a short join truncates stamp coverage.
        self.q.put(None)
        self._stop.set()
        self.writer.join()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--algo", default="")
    ap.add_argument("--topic", default="")
    ap.add_argument("--out-dir", required=True, help="metrics/meta dir (usually results/...)")
    ap.add_argument(
        "--cloud-dir",
        default="",
        help="directory for cloud_merged_ds.txt (default: same as --out-dir)",
    )
    ap.add_argument(
        "--leaf",
        type=float,
        default=0.0,
        help="voxel leaf size (m); <=0 disables downsample (default 0)",
    )
    ap.add_argument("--every-n", type=int, default=1, help="keep every Nth published cloud")
    ap.add_argument("--merge", action="store_true", default=True)
    ap.add_argument("--no-merge", action="store_false", dest="merge")
    ap.add_argument("--save-frames", action="store_true", help="also write per-frame txt")
    ap.add_argument("--stop-file", default="")
    ap.add_argument("--max-sec", type=float, default=0.0)
    args = ap.parse_args()

    topic = args.topic or DEFAULT_TOPICS.get(args.algo, "")
    if not topic:
        print("Need --topic or known --algo", file=sys.stderr)
        return 2

    out_dir = Path(args.out_dir)
    cloud_dir = Path(args.cloud_dir) if args.cloud_dir else out_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    cloud_dir.mkdir(parents=True, exist_ok=True)

    merge_path = (cloud_dir / "cloud_merged_ds.txt") if args.merge else None

    rospy.init_node("lio_save_clouds", anonymous=True, disable_signals=True)
    rec = CloudRecorder(args.leaf, args.every_n, merge_path)
    # Large queue: offline publishers burst faster than real-time.
    sub = rospy.Subscriber(topic, PointCloud2, rec.cb, queue_size=5000, buff_size=2**28, tcp_nodelay=True)

    stop = False

    def _stop(*_):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    t0 = time.time()
    rate = rospy.Rate(50)
    while not stop and not rospy.is_shutdown():
        if args.stop_file and Path(args.stop_file).exists():
            break
        if args.max_sec > 0 and (time.time() - t0) > args.max_sec:
            break
        rate.sleep()

    # Drain in-flight publishes then finish writer
    time.sleep(1.0)
    sub.unregister()
    rec.finish()

    with rec.lock:
        frames_meta = list(rec.frames_meta)

    index = [
        {"i": i, "stamp": stamp, "n_points": n}
        for i, (stamp, n) in enumerate(frames_meta)
    ]

    merge_file = "cloud_merged_ds.txt" if (args.merge and merge_path and merge_path.exists()) else None
    stamp0 = frames_meta[0][0] if frames_meta else None
    stampN = frames_meta[-1][0] if frames_meta else None
    meta = {
        "algo": args.algo,
        "topic": topic,
        "leaf_m": args.leaf,
        "downsample": bool(args.leaf and args.leaf > 0),
        "every_n": args.every_n,
        "n_msgs_seen": rec.n_raw,
        "n_frames_kept": len(frames_meta),
        "n_queue_dropped": rec.n_dropped,
        "n_points_raw_sum": rec.n_pts_raw,
        "n_points_saved": rec.n_pts_saved,
        "merged_file": merge_file,
        "merged_points": rec.n_pts_saved if merge_file else 0,
        "cloud_dir": str(cloud_dir),
        "merged_path": str(merge_path) if merge_file else None,
        "stamp_first": stamp0,
        "stamp_last": stampN,
        "stamp_span_sec": (stampN - stamp0) if (stamp0 is not None and stampN is not None) else None,
        "save_frames": args.save_frames,
    }
    (out_dir / "clouds_meta.json").write_text(json.dumps(meta, indent=2))
    (out_dir / "clouds_index.json").write_text(json.dumps(index, indent=2))
    if cloud_dir != out_dir:
        (cloud_dir / "clouds_meta.json").write_text(json.dumps(meta, indent=2))
    print(json.dumps(meta, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
