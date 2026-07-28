#!/usr/bin/env python3
"""Parse per-frame timing hints from algo logs (best-effort)."""
from __future__ import annotations

import argparse
import json
import re
import statistics
import sys
from pathlib import Path

# Common patterns seen across FAST-LIO family / Faster-LIO / Super-LIO logs
PATTERNS = [
    re.compile(r"Laser Mapping Single Run\s*\]\s*average time usage:\s*([0-9.]+)\s*ms", re.I),
    re.compile(r"Laser Mapping Single Run[:\s]+([0-9.]+)\s*ms", re.I),
    re.compile(r"average time usage:\s*([0-9.]+)\s*ms", re.I),
    re.compile(r"mean\s+time[:\s]+([0-9.]+)\s*ms", re.I),
    re.compile(r"\[Timing\].*?([0-9.]+)\s*ms", re.I),
    re.compile(r"whole mapping time.*?([0-9.]+)", re.I),
    re.compile(r"frame\s+time[:\s]+([0-9.]+)", re.I),
    re.compile(r"cost\s+time[:\s]+([0-9.]+)\s*ms", re.I),
]


def parse_file(path: Path) -> dict:
    text = path.read_text(errors="ignore")
    vals = []
    for pat in PATTERNS:
        vals.extend(float(x) for x in pat.findall(text))
    out = {"log": str(path), "n_matches": len(vals)}
    if vals:
        out.update(
            {
                "frame_ms_mean": statistics.mean(vals),
                "frame_ms_median": statistics.median(vals),
                "frame_ms_p95": sorted(vals)[int(0.95 * (len(vals) - 1))],
                "frame_ms_max": max(vals),
            }
        )
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
