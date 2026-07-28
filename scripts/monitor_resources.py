#!/usr/bin/env python3
"""Sample CPU% and RSS for a process tree; write CSV and summary JSON."""
from __future__ import annotations

import argparse
import csv
import json
import os
import signal
import sys
import time
from pathlib import Path

try:
    import psutil
except ImportError:
    print("psutil required: pip install psutil", file=sys.stderr)
    sys.exit(1)


def collect_pids(root_pid: int) -> list:
    try:
        root = psutil.Process(root_pid)
    except psutil.NoSuchProcess:
        return []
    pids = [root]
    try:
        pids.extend(root.children(recursive=True))
    except psutil.Error:
        pass
    return pids


def sample(pids) -> tuple:
    cpu = 0.0
    rss = 0
    alive = []
    for p in pids:
        try:
            cpu += p.cpu_percent(interval=None)
            rss += p.memory_info().rss
            alive.append(p)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    return cpu, rss, alive


def busy_cpu_percent(pids, prev_times, dt: float):
    """Fallback CPU% from cpu_times delta across the process tree."""
    total = 0.0
    new_times = {}
    for p in pids:
        try:
            t = p.cpu_times()
            cur = t.user + t.system
            new_times[p.pid] = cur
            if p.pid in prev_times and dt > 0:
                total += max(0.0, cur - prev_times[p.pid])
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    pct = (total / dt) * 100.0 if dt > 0 else 0.0
    return pct, new_times


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pid", type=int, required=True)
    ap.add_argument("--out", required=True, help="output CSV path")
    ap.add_argument("--summary", required=True, help="output JSON summary path")
    ap.add_argument("--interval", type=float, default=0.5)
    ap.add_argument("--stop-file", default="", help="exit when this file appears")
    args = ap.parse_args()

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    summary_path = Path(args.summary)
    summary_path.parent.mkdir(parents=True, exist_ok=True)

    stop = False

    def _stop(*_):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    pids = collect_pids(args.pid)
    for p in pids:
        try:
            p.cpu_percent(interval=None)
        except psutil.Error:
            pass

    rows = []
    peak_rss = 0
    peak_cpu = 0.0
    sum_cpu = 0.0
    t0 = time.time()
    prev_times = {}
    last_t = t0

    with out.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["elapsed_sec", "cpu_percent", "rss_bytes", "n_procs"])
        while not stop:
            if args.stop_file and os.path.exists(args.stop_file):
                break
            pids = collect_pids(args.pid)
            if not pids:
                break
            now = time.time()
            dt = now - last_t
            cpu, rss, pids = sample(pids)
            busy, prev_times = busy_cpu_percent(pids, prev_times, dt)
            if cpu <= 0.0 and busy > 0.0:
                cpu = busy
            last_t = now
            elapsed = now - t0
            w.writerow([f"{elapsed:.3f}", f"{cpu:.2f}", rss, len(pids)])
            f.flush()
            rows.append((elapsed, cpu, rss))
            peak_rss = max(peak_rss, rss)
            peak_cpu = max(peak_cpu, cpu)
            sum_cpu += cpu
            time.sleep(args.interval)

    n = max(len(rows), 1)
    mean_cpu = sum_cpu / n
    mean_rss = sum(r[2] for r in rows) / n if rows else 0
    summary = {
        "pid": args.pid,
        "samples": len(rows),
        "duration_sec": rows[-1][0] if rows else 0.0,
        "cpu_percent_mean": mean_cpu,
        "cpu_percent_peak": peak_cpu,
        "rss_bytes_mean": mean_rss,
        "rss_bytes_peak": peak_rss,
        "rss_mb_peak": peak_rss / (1024 * 1024),
    }
    summary_path.write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary))
    return 0


if __name__ == "__main__":
    sys.exit(main())
