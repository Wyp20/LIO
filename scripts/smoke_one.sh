#!/usr/bin/env bash
# Smoke one bag through a small set of algorithms.
# Full bags are huge; by default we trim first MAX_SEC seconds of lidar+imu.
set -euo pipefail

LIO_WS="${LIO_WS:-/home/wyp/Project_lidar_navigation/LIO}"
export LIO_WS
SRC_BAG="${1:-/media/wyp/娱乐/qiao_lidar_data/auto_record_20260716_075831.bag}"
MAX_SEC="${MAX_SEC:-20}"
ALGOS="${ALGOS:-faster_lio fast_lio bievr}"

echo "[smoke] src_bag=$SRC_BAG max_sec=$MAX_SEC algos=$ALGOS"
mkdir -p "${LIO_WS}/results/smoke"

STEM="$(basename "$SRC_BAG" .bag)"
TRIM_BAG="${LIO_WS}/results/smoke/${STEM}_${MAX_SEC}s.bag"
if [[ ! -f "$TRIM_BAG" ]]; then
  echo "[smoke] trimming ${MAX_SEC}s lidar+imu -> $TRIM_BAG"
  source /opt/ros/noetic/setup.bash
  python3 - <<PY
import rosbag
src = r"$SRC_BAG"
dst = r"$TRIM_BAG"
max_sec = float("$MAX_SEC")
topics = ["/lidar_points", "/fvs/imu_raw"]
in_bag = rosbag.Bag(src, "r")
out = rosbag.Bag(dst, "w")
t0 = None
n = 0
for topic, msg, t in in_bag.read_messages(topics=topics):
    if t0 is None:
        t0 = t.to_sec()
    if t.to_sec() - t0 > max_sec:
        break
    out.write(topic, msg, t)
    n += 1
in_bag.close()
out.close()
print(f"wrote {dst} msgs={n}")
PY
fi

FAIL=0
for a in $ALGOS; do
  echo "======== smoke $a ========"
  if ! "${LIO_WS}/scripts/run_one.sh" --algo "$a" --bag "$TRIM_BAG" --max-sec 0 \
      --out "${LIO_WS}/results/smoke/${a}"; then
    echo "[smoke] FAIL $a"
    FAIL=$((FAIL + 1))
  else
    echo "[smoke] OK $a"
  fi
done

echo "[smoke] failures=$FAIL"
python3 - <<PY
import json
from pathlib import Path
root = Path("${LIO_WS}/results/smoke")
rows = []
for d in sorted(root.iterdir()):
    if not d.is_dir():
        continue
    meta, res, timing = {}, {}, {}
    try: meta = json.loads((d/"meta.json").read_text())
    except Exception: pass
    try: res = json.loads((d/"resources.json").read_text())
    except Exception: pass
    try: timing = json.loads((d/"timing.json").read_text())
    except Exception: pass
    rows.append({
        "algo": d.name,
        "exit_code": meta.get("exit_code"),
        "cpu_peak": res.get("cpu_percent_peak"),
        "rss_mb_peak": res.get("rss_mb_peak"),
        "frame_ms_mean": timing.get("frame_ms_mean") if isinstance(timing, dict) else None,
        "skipped": res.get("skipped", False),
    })
out = root/"summary.json"
out.write_text(json.dumps(rows, indent=2))
print(out.read_text())
PY

exit "$FAIL"
