#!/usr/bin/env bash
# Single-thread offline eval for Hesai bag window [15s, 155s].
# Dense publish + C++ accumulate then PCL ASCII .txt dump to CLOUD_ROOT.
set -euo pipefail

LIO_WS="${LIO_WS:-/home/wyp/Project_lidar_navigation/LIO}"
export LIO_WS

BAG="${1:-/media/wyp/娱乐/qiao_lidar_data/auto_record_20260716_075831.bag}"
# Fixed eval window (do not inherit polluted env START_SEC/MAX_SEC)
START_SEC=15
MAX_SEC=140
ALGOS="${ALGOS:-faster_lio fast_lio point_lio pv_lio akf_lio voxel_map_plus rvoxelmap super_lio dlio bievr}"
CPUS="${CPUS:-0}"
SAVE_CLOUDS=1
CLOUD_LEAF=0
CLOUD_EVERY_N=1
CLOUD_ROOT="${CLOUD_ROOT:-/media/wyp/娱乐/lio_eval_clouds}"

STEM="$(basename "$BAG" .bag)"
TAG="${STEM}_${START_SEC}to$((START_SEC + MAX_SEC))s_1t_dense_full"
OUT_ROOT="${LIO_WS}/results/eval_1thread/${TAG}"
CLOUD_TAG="$TAG"
mkdir -p "$OUT_ROOT"
mkdir -p "${CLOUD_ROOT}/${CLOUD_TAG}"

echo "[eval] bag=$BAG window=${START_SEC}s+${MAX_SEC}s algos=$ALGOS"
echo "[eval] metrics=$OUT_ROOT"
echo "[eval] clouds=${CLOUD_ROOT}/${CLOUD_TAG} (leaf=$CLOUD_LEAF every_n=$CLOUD_EVERY_N)"

FAIL=0
for a in $ALGOS; do
  echo "======== eval $a ========"
  if ! env -u START_SEC -u MAX_SEC -u SAVE_CLOUDS -u LIO_BAG_START_SEC -u LIO_BAG_DURATION_SEC \
      CLOUD_LEAF="$CLOUD_LEAF" CLOUD_EVERY_N="$CLOUD_EVERY_N" CLOUD_ROOT="$CLOUD_ROOT" \
      "${LIO_WS}/scripts/run_one.sh" \
      --algo "$a" --bag "$BAG" \
      --start-sec "$START_SEC" --max-sec "$MAX_SEC" \
      --cpus "$CPUS" --save-clouds "$SAVE_CLOUDS" \
      --cloud-dir "${CLOUD_ROOT}/${CLOUD_TAG}/${a}" \
      --out "${OUT_ROOT}/${a}"; then
    echo "[eval] FAIL $a"
    FAIL=$((FAIL + 1))
  else
    echo "[eval] OK $a"
  fi
done

python3 - <<PY
import json, csv, re, statistics
from pathlib import Path
root = Path("${OUT_ROOT}")
rows = []
for d in sorted(p for p in root.iterdir() if p.is_dir()):
    s = {}
    try:
        s = json.loads((d / "summary.json").read_text())
    except Exception:
        s = {"algo": d.name, "exit_code": -1}
    log = d / "run.log"
    if log.exists():
        vals = [float(x) for x in re.findall(r"\[Frame Time\]\s*([0-9.]+)\s*ms", log.read_text(errors="ignore"))]
        if vals:
            s["frame_ms_mean"] = statistics.mean(vals)
            s["frame_ms_max"] = max(vals)
            s["frame_ms_median"] = statistics.median(vals)
            s["n_frames_timing"] = len(vals)
    s.setdefault("algo", d.name)
    rows.append(s)

(root / "summary_all.json").write_text(json.dumps(rows, indent=2))
fields = [
    "algo", "exit_code",
    "cpu_percent_mean", "cpu_percent_peak",
    "mem_percent_mean", "mem_percent_peak",
    "rss_mb_mean", "rss_mb_peak",
    "frame_ms_mean", "frame_ms_max", "frame_ms_median", "n_frames_timing",
    "cloud_msgs_seen", "cloud_frames_kept", "cloud_points_saved",
    "cloud_dir", "cloud_merged_path",
]
with (root / "summary_all.csv").open("w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
    w.writeheader()
    for r in rows:
        w.writerow(r)

def fmt(r):
    def r2(k, nd=2):
        v = r.get(k)
        return None if v is None else round(float(v), nd)
    return {
        "algo": r.get("algo"),
        "cpu_percent_mean": r2("cpu_percent_mean"),
        "mem_percent_mean": r2("mem_percent_mean", 3),
        "rss_mb_peak": r2("rss_mb_peak"),
        "frame_ms_mean": r2("frame_ms_mean", 3),
        "frame_ms_max": r2("frame_ms_max", 3),
        "n_frames": r.get("n_frames_timing"),
        "exit_code": r.get("exit_code"),
    }
fmt_rows = [fmt(r) for r in rows]
print("\n| algo | CPU% mean | mem% mean | RSS peak MB | avg ms | max ms | frames |")
print("|------|-----------|-----------|-------------|--------|--------|--------|")
for r in sorted(fmt_rows, key=lambda x: x["algo"] or ""):
    print(f"| {r['algo']} | {r['cpu_percent_mean']} | {r['mem_percent_mean']} | {r['rss_mb_peak']} | {r['frame_ms_mean']} | {r['frame_ms_max']} | {r['n_frames']} |")
print(f"\nCSV -> {root / 'summary_all.csv'}")
print(f"clouds -> ${CLOUD_ROOT}/${CLOUD_TAG}")
PY

echo "[eval] failures=$FAIL -> $OUT_ROOT"
exit "$FAIL"
