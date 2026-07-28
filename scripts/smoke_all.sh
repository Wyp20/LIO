#!/usr/bin/env bash
# Smoke all algos on a time window of auto_record_20260716_075831.bag
# Default window: 15s .. 155s (relative to bag start)
set -euo pipefail

LIO_WS="${LIO_WS:-/home/wyp/Project_lidar_navigation/LIO}"
export LIO_WS
SRC_BAG="${1:-/media/wyp/娱乐/qiao_lidar_data/auto_record_20260716_075831.bag}"
START_SEC="${START_SEC:-15}"
END_SEC="${END_SEC:-155}"
ALGOS="${ALGOS:-faster_lio fast_lio point_lio pv_lio akf_lio voxel_map_plus rvoxelmap super_lio dlio bievr}"

echo "[smoke_all] src=$SRC_BAG window=${START_SEC}s..${END_SEC}s"
mkdir -p "${LIO_WS}/results/smoke_all"
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1091
source "${LIO_WS}/devel/setup.bash"

STEM="$(basename "$SRC_BAG" .bag)"
TRIM_BAG="${LIO_WS}/results/smoke_all/${STEM}_${START_SEC}s_${END_SEC}s.bag"
if [[ ! -f "$TRIM_BAG" ]]; then
  echo "[smoke_all] trimming ${START_SEC}s..${END_SEC}s lidar+imu -> $TRIM_BAG"
  python3 - <<PY
import rosbag
src, dst = r"$SRC_BAG", r"$TRIM_BAG"
start_sec, end_sec = float("$START_SEC"), float("$END_SEC")
topics = ["/lidar_points", "/fvs/imu_raw"]
inn, out = rosbag.Bag(src, "r"), rosbag.Bag(dst, "w")
t0 = None
n = 0
for topic, msg, t in inn.read_messages(topics=topics):
    ts = t.to_sec()
    if t0 is None:
        t0 = ts
    rel = ts - t0
    if rel < start_sec:
        continue
    if rel > end_sec:
        break
    out.write(topic, msg, t)
    n += 1
inn.close()
out.close()
print(f"msgs={n} duration={end_sec-start_sec}s")
PY
fi

FAIL=0
OK=0
SKIP=0
for a in $ALGOS; do
  echo "======== $a ========"
  if "${LIO_WS}/scripts/run_one.sh" --algo "$a" --bag "$TRIM_BAG" --max-sec 0 \
      --out "${LIO_WS}/results/smoke_all/${a}"; then
    echo "[OK] $a"; OK=$((OK+1))
  else
    rc=$?
    if [[ $rc -eq 2 ]]; then
      echo "[SKIP] $a"; SKIP=$((SKIP+1))
    else
      echo "[FAIL] $a rc=$rc"; FAIL=$((FAIL+1))
    fi
  fi
done

python3 - <<PY
import json
from pathlib import Path
root = Path("${LIO_WS}/results/smoke_all")
rows = []
for d in sorted(p for p in root.iterdir() if p.is_dir()):
    meta, res = {}, {}
    try: meta = json.loads((d/"meta.json").read_text())
    except Exception: pass
    try: res = json.loads((d/"resources.json").read_text())
    except Exception: pass
    rows.append({
        "algo": d.name,
        "exit_code": meta.get("exit_code"),
        "duration_sec": res.get("duration_sec"),
        "cpu_peak": res.get("cpu_percent_peak"),
        "rss_mb_peak": res.get("rss_mb_peak"),
        "skipped": res.get("skipped", False),
    })
(root/"summary.json").write_text(json.dumps(rows, indent=2))
print((root/"summary.json").read_text())
PY

echo "[smoke_all] ok=$OK fail=$FAIL skip=$SKIP"
exit "$FAIL"
