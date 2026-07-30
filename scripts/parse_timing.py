#!/usr/bin/env python3
"""Parse per-frame timing from algo logs. Prefer unified '[Frame Time] X ms' lines."""
from __future__ import annotations

import argparse
import csv
import json
import re
import statistics
import sys
from pathlib import Path

FRAME_TIME_RE = re.compile(
    r"\[Frame Time\](?:\s*Frame\s+\d+\s+processing time:)?\s*([0-9.]+)\s*ms",
    re.I,
)


def stats_from(vals: list) -> dict:
    filtered = [v for v in vals if 0.01 < v < 5000.0] or list(vals)
    if not filtered:
        return {}
    return {
        "frame_ms_mean": statistics.mean(filtered),
        "frame_ms_median": statistics.median(filtered),
        "frame_ms_p95": sorted(filtered)[int(0.95 * (len(filtered) - 1))],
        "frame_ms_max": max(filtered),
        "frame_ms_min": min(filtered),
        "n_frames": len(filtered),
    }


def parse_time_log(path: Path) -> dict:
    if not path.exists():
        return {}
    text = path.read_text(errors="ignore").strip()
    if not text:
        return {}
    lines = [ln for ln in text.splitlines() if ln.strip()]
    try:
        reader = csv.reader(lines)
        rows = list(reader)
        if len(rows) >= 2:
            header = [h.strip().lower() for h in rows[0]]
            idx = None
            for key in ("laser mapping single run", "total", "whole"):
                for i, h in enumerate(header):
                    if key in h:
                        idx = i
                        break
                if idx is not None:
                    break
            if idx is None:
                idx = len(header) - 1
            vals = []
            for r in rows[1:]:
                if len(r) > idx:
                    try:
                        vals.append(float(r[idx]))
                    except ValueError:
                        pass
            if vals:
                out = stats_from(vals)
                out["source"] = "time.log"
                return out
    except Exception:
        pass
    return {}


def parse_file(path: Path) -> dict:
    text = path.read_text(errors="ignore")
    out = {"log": str(path), "n_matches": 0}

    # 1) Unified per-frame lines
    vals = [float(x) for x in FRAME_TIME_RE.findall(text)]
    if vals:
        st = stats_from(vals)
        out.update(st)
        out["source"] = "frame_time_log"
        out["n_matches"] = st.get("n_frames", 0)
        return out

    # 2) faster-lio time.log companion
    tlog = parse_time_log(path.parent / "time.log")
    if tlog.get("n_frames"):
        out.update(tlog)
        out["n_matches"] = tlog["n_frames"]
        return out

    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("log", nargs="+")
    ap.add_argument("--out", default="")
    args = ap.parse_args()
    results = [parse_file(Path(p)) for p in args.log]
    text = json.dumps(results if len(results) > 1 else results[0], indent=2)
    if args.out:
        Path(args.out).write_text(text)
    print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
