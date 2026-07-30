#!/usr/bin/env bash
# Run one (algo, bag) pair under single-core limits and record resources + clouds.
set -euo pipefail

LIO_WS="${LIO_WS:-/home/wyp/Project_lidar_navigation/LIO}"
export LIO_WS
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1091
source "${LIO_WS}/devel/setup.bash"

ALGO=""
BAG=""
START_SEC="${START_SEC:-0}"
MAX_SEC=0
OUT_DIR=""
CPUS="${CPUS:-0}"
SAVE_CLOUDS="${SAVE_CLOUDS:-1}"
CLOUD_LEAF="${CLOUD_LEAF:-0}"
CLOUD_EVERY_N="${CLOUD_EVERY_N:-1}"
CLOUD_ROOT="${CLOUD_ROOT:-/media/wyp/娱乐/lio_eval_clouds}"
CLOUD_DIR=""

usage() {
  cat <<EOF
Usage: $0 --algo <name> --bag <path> [--start-sec N] [--max-sec N] [--out DIR] [--cpus 0]

Algos:
  fast_lio faster_lio point_lio pv_lio akf_lio voxel_map_plus
  rvoxelmap super_lio dlio bievr rko_lio

Env:
  SAVE_CLOUDS=1 CLOUD_LEAF=0 CLOUD_EVERY_N=1
  CLOUD_ROOT=/media/wyp/娱乐/lio_eval_clouds  (dense clouds saved here)
  LIO_BAG_START_SEC / LIO_BAG_DURATION_SEC (also set from --start-sec/--max-sec)
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --algo) ALGO="$2"; shift 2 ;;
    --bag) BAG="$2"; shift 2 ;;
    --start-sec) START_SEC="$2"; shift 2 ;;
    --max-sec) MAX_SEC="$2"; shift 2 ;;
    --out) OUT_DIR="$2"; shift 2 ;;
    --cpus) CPUS="$2"; shift 2 ;;
    --save-clouds) SAVE_CLOUDS="$2"; shift 2 ;;
    --cloud-root) CLOUD_ROOT="$2"; shift 2 ;;
    --cloud-dir) CLOUD_DIR="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown arg: $1"; usage; exit 1 ;;
  esac
done

[[ -n "$ALGO" && -n "$BAG" ]] || { usage; exit 1; }
[[ -f "$BAG" ]] || { echo "bag not found: $BAG"; exit 1; }

BAG_STEM="$(basename "$BAG" .bag)"
OUT_DIR="${OUT_DIR:-${LIO_WS}/results/${ALGO}/${BAG_STEM}}"
mkdir -p "$OUT_DIR"
# Dense clouds on external disk by default
if [[ -z "$CLOUD_DIR" ]]; then
  CLOUD_TAG="${BAG_STEM}_${START_SEC}to$((START_SEC + MAX_SEC))s_1t_dense"
  CLOUD_DIR="${CLOUD_ROOT}/${CLOUD_TAG}/${ALGO}"
fi
mkdir -p "$CLOUD_DIR"
STOP_FILE="${OUT_DIR}/.stop_monitor"
STOP_CLOUD="${OUT_DIR}/.stop_clouds"
rm -f "$STOP_FILE" "$STOP_CLOUD"

# Single-thread runtime limits
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export TBB_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1
export LIO_BAG_START_SEC="$START_SEC"
export LIO_BAG_DURATION_SEC="$MAX_SEC"
# C++ saver accumulates in RAM then writes once; no publisher pacing needed.
# Override with LIO_CLOUD_PACE_MS>0 only if a subscriber still drops messages.
if [[ "$SAVE_CLOUDS" == "1" ]]; then
  export LIO_CLOUD_PACE_MS="${LIO_CLOUD_PACE_MS:-0}"
fi

LOG="${OUT_DIR}/run.log"
RES_CSV="${OUT_DIR}/resources.csv"
RES_JSON="${OUT_DIR}/resources.json"
TIMING_JSON="${OUT_DIR}/timing.json"
META_JSON="${OUT_DIR}/meta.json"
SUMMARY_JSON="${OUT_DIR}/summary.json"
: > "$LOG"

echo "[run_one] algo=$ALGO bag=$BAG start=$START_SEC max_sec=$MAX_SEC out=$OUT_DIR clouds=$CLOUD_DIR cpus=$CPUS threads=1 leaf=$CLOUD_LEAF every_n=$CLOUD_EVERY_N"

ensure_roscore() {
  if ! rostopic list >/dev/null 2>&1; then
    echo "[run_one] starting roscore"
    taskset -c "$CPUS" roscore >/dev/null 2>&1 &
    echo $! > "${OUT_DIR}/roscore.pid"
    for _ in $(seq 1 50); do
      rostopic list >/dev/null 2>&1 && break
      sleep 0.2
    done
  fi
}

# Kill leftover mapping nodes so the next launch is not "same name" preempted.
clear_stale_nodes() {
  ensure_roscore
  local names
  names="$(rosnode list 2>/dev/null | grep -E 'laserMapping|laser_mapping|dlio|super_lio|bievr' || true)"
  if [[ -n "$names" ]]; then
    echo "[run_one] clearing stale nodes:"
    echo "$names"
    # shellcheck disable=SC2086
    echo "$names" | xargs -r -n1 rosnode kill >/dev/null 2>&1 || true
    sleep 1
  fi
}

set_bag_window_params() {
  ensure_roscore
  clear_stale_nodes
  rosparam set /bag_start_sec "$START_SEC" >/dev/null
  rosparam set /bag_duration_sec "$MAX_SEC" >/dev/null
}

start_monitor() {
  local pid="$1"
  python3 "${LIO_WS}/scripts/monitor_resources.py" \
    --pid "$pid" --out "$RES_CSV" --summary "$RES_JSON" \
    --interval 0.5 --stop-file "$STOP_FILE" >"${OUT_DIR}/monitor.log" 2>&1 &
  echo $! > "${OUT_DIR}/monitor.pid"
}

start_cloud_saver() {
  [[ "$SAVE_CLOUDS" == "1" ]] || return 0
  ensure_roscore
  rm -f "$STOP_CLOUD"
  # Prefer C++ accumulator (PCL ASCII .txt once at end); fall back to python.
  local saver_bin
  saver_bin="$(rospack find lio_cloud_saver 2>/dev/null)/../../devel/lib/lio_cloud_saver/save_clouds_node"
  if [[ ! -x "$saver_bin" ]]; then
    saver_bin="${LIO_WS}/devel/lib/lio_cloud_saver/save_clouds_node"
  fi
  if [[ -x "$saver_bin" ]]; then
    echo "[run_one] cloud saver: C++ save_clouds_node -> ${CLOUD_DIR}/cloud_merged.txt"
    # Do not pin saver to the algo CPU — accumulate/save should not contend with timing.
    "$saver_bin" \
      __name:="lio_save_clouds_${ALGO}" \
      _algo:="$ALGO" \
      _out_dir:="$OUT_DIR" \
      _cloud_dir:="$CLOUD_DIR" \
      _stop_file:="$STOP_CLOUD" \
      >"${OUT_DIR}/clouds_saver.log" 2>&1 &
  else
    echo "[run_one] WARN: save_clouds_node missing, fallback python save_clouds.py"
    python3 "${LIO_WS}/scripts/save_clouds.py" \
      --algo "$ALGO" --out-dir "$OUT_DIR" --cloud-dir "$CLOUD_DIR" \
      --leaf "$CLOUD_LEAF" --every-n "$CLOUD_EVERY_N" --merge \
      --stop-file "$STOP_CLOUD" \
      >"${OUT_DIR}/clouds_saver.log" 2>&1 &
  fi
  echo $! > "${OUT_DIR}/clouds_saver.pid"
  # Wait until subscriber is ready (offline nodes publish very fast).
  sleep 2
}

stop_cloud_saver() {
  touch "$STOP_CLOUD" || true
  if [[ -f "${OUT_DIR}/clouds_saver.pid" ]]; then
    local cpid
    cpid="$(cat "${OUT_DIR}/clouds_saver.pid")"
    # PCL ASCII dump of dense clouds can take a while on USB; wait generously.
    local i=0
    while kill -0 "$cpid" 2>/dev/null; do
      sleep 1
      i=$((i + 1))
      if [[ $i -ge 1800 ]]; then
        echo "[run_one] cloud saver still running after ${i}s, sending SIGTERM"
        kill -TERM "$cpid" 2>/dev/null || true
        break
      fi
      if [[ $((i % 30)) -eq 0 ]]; then
        echo "[run_one] waiting cloud saver... ${i}s"
      fi
    done
    wait "$cpid" 2>/dev/null || true
  fi
}

stop_monitor() {
  touch "$STOP_FILE"
  if [[ -f "${OUT_DIR}/monitor.pid" ]]; then
    local mpid
    mpid="$(cat "${OUT_DIR}/monitor.pid")"
    wait "$mpid" 2>/dev/null || true
  fi
}

cleanup() {
  touch "$STOP_FILE" "$STOP_CLOUD" || true
  if [[ -n "${ALGO_PGID:-}" ]]; then
    kill -TERM -"$ALGO_PGID" 2>/dev/null || true
  fi
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill "$LAUNCH_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

python3 - <<PY
import json, time
from pathlib import Path
meta = {
  "algo": "$ALGO",
  "bag": "$BAG",
  "start_sec": float("$START_SEC"),
  "max_sec": float("$MAX_SEC"),
  "cpus": "$CPUS",
  "omp_num_threads": 1,
  "save_clouds": int("$SAVE_CLOUDS"),
  "cloud_leaf": float("$CLOUD_LEAF"),
  "cloud_every_n": int("$CLOUD_EVERY_N"),
  "cloud_dir": "$CLOUD_DIR",
  "started_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
  "config_dir": "${LIO_WS}/configs/hesai",
}
Path("$META_JSON").write_text(json.dumps(meta, indent=2))
PY

RC=1

case "$ALGO" in
  faster_lio)
    ensure_roscore
    set_bag_window_params
    CFG="${LIO_WS}/configs/hesai/faster_lio.yaml"
    BIN="${LIO_WS}/devel/lib/faster_lio/run_mapping_offline"
    [[ -x "$BIN" ]] || BIN="$(find "${LIO_WS}/devel" -name run_mapping_offline -type f | head -1)"
    [[ -x "$BIN" ]] || { echo "run_mapping_offline not found"; exit 1; }
    TRAJ="${OUT_DIR}/traj.txt"
    TLOG="${OUT_DIR}/time.log"
    start_cloud_saver
    set +e
    taskset -c "$CPUS" "$BIN" \
      --config_file="$CFG" \
      --bag_file="$BAG" \
      --traj_log_file="$TRAJ" \
      --time_log_file="$TLOG" \
      --bag_start_sec="$START_SEC" \
      --bag_duration_sec="$MAX_SEC" \
      >>"$LOG" 2>&1 &
    ALGO_PID=$!
    start_monitor "$ALGO_PID"
    wait "$ALGO_PID"
    RC=$?
    set -e
    stop_monitor
    stop_cloud_saver
    ;;

  bievr)
    ensure_roscore
    set_bag_window_params
    export LD_LIBRARY_PATH="${HOME}/.local/lib:${HOME}/.cache/ceres-build/build/lib:${LD_LIBRARY_PATH:-}"
    start_cloud_saver
    set +e
    taskset -c "$CPUS" roslaunch "${LIO_WS}/launch/hesai/bievr.launch" \
      rosbag:="$BAG" \
      rviz:=false \
      sensor_config:="${LIO_WS}/configs/hesai/bievr_sensor.yaml" \
      params:="${LIO_WS}/configs/hesai/bievr_params.yaml" \
      >>"$LOG" 2>&1 &
    LAUNCH_PID=$!
    start_monitor "$LAUNCH_PID"
    wait "$LAUNCH_PID"
    RC=$?
    set -e
    stop_monitor
    stop_cloud_saver
    if grep -qE 'error while loading shared libraries|Segmentation fault|Aborted \(core dumped\)' "$LOG"; then
      RC=1
    elif grep -q 'has finished cleanly' "$LOG" || grep -q 'Done with for loop' "$LOG"; then
      RC=0
    fi
    ;;

  rko_lio)
    echo "[run_one] rko_lio requires conda Python>=3.10 (pending). Skipping." | tee -a "$LOG"
    RC=2
    python3 - <<PY
import json
from pathlib import Path
Path("$RES_JSON").write_text(json.dumps({"skipped": True, "reason": "rko conda pending"}, indent=2))
PY
    ;;

  fast_lio|point_lio|pv_lio|akf_lio|voxel_map_plus|rvoxelmap|super_lio|dlio)
    ensure_roscore
    set_bag_window_params
    LAUNCH_FILE="${LIO_WS}/launch/hesai/${ALGO}.launch"
    start_cloud_saver
    set +e
    taskset -c "$CPUS" roslaunch "$LAUNCH_FILE" bag_path:="$BAG" rviz:=false >>"$LOG" 2>&1 &
    LAUNCH_PID=$!
    start_monitor "$LAUNCH_PID"
    wait "$LAUNCH_PID"
    RC=$?
    set -e
    stop_monitor
    stop_cloud_saver
    if grep -qE 'Segmentation fault|Aborted \(core dumped\)|Failed to open bag|exit code -11|process has died' "$LOG"; then
      RC=1
    elif grep -qE 'Offline bag opened|OFFLINE|offline bag' "$LOG"; then
      if ! grep -qE 'process has died|exit code -' "$LOG"; then
        [[ "$RC" -ne 0 ]] && RC=0
      else
        RC=1
      fi
    fi
    ;;

  *)
    echo "Unknown algo: $ALGO"; exit 1
    ;;
esac

python3 "${LIO_WS}/scripts/parse_timing.py" "$LOG" --out "$TIMING_JSON" >/dev/null 2>&1 || true

python3 - <<PY
import json, time
from pathlib import Path

out = Path("$OUT_DIR")
meta = {}
res = {}
timing = {}
clouds = {}
try:
    meta = json.loads((out / "meta.json").read_text())
except Exception:
    pass
try:
    res = json.loads((out / "resources.json").read_text())
except Exception:
    pass
try:
    timing = json.loads((out / "timing.json").read_text())
except Exception:
    pass
try:
    clouds = json.loads((out / "clouds_meta.json").read_text())
except Exception:
    pass

meta["finished_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")
meta["exit_code"] = int("$RC")
meta["out_dir"] = str(out)
(out / "meta.json").write_text(json.dumps(meta, indent=2))

summary = {
    "algo": meta.get("algo", "$ALGO"),
    "bag": meta.get("bag", "$BAG"),
    "start_sec": meta.get("start_sec"),
    "max_sec": meta.get("max_sec"),
    "exit_code": meta.get("exit_code"),
    "omp_num_threads": 1,
    "cpu_percent_mean": res.get("cpu_percent_mean"),
    "cpu_percent_peak": res.get("cpu_percent_peak"),
    "mem_percent_mean": res.get("mem_percent_mean"),
    "mem_percent_peak": res.get("mem_percent_peak"),
    "rss_mb_mean": res.get("rss_mb_mean"),
    "rss_mb_peak": res.get("rss_mb_peak"),
    "frame_ms_mean": timing.get("frame_ms_mean"),
    "frame_ms_max": timing.get("frame_ms_max"),
    "frame_ms_median": timing.get("frame_ms_median"),
    "n_frames_timing": timing.get("n_frames") or timing.get("n_matches"),
    "cloud_merged": clouds.get("merged_file"),
    "cloud_dir": clouds.get("cloud_dir") or "$CLOUD_DIR",
    "cloud_merged_path": clouds.get("merged_path"),
    "cloud_frames_kept": clouds.get("n_frames_kept"),
    "cloud_msgs_seen": clouds.get("n_msgs_seen"),
    "cloud_points_saved": clouds.get("n_points_saved"),
}
(out / "summary.json").write_text(json.dumps(summary, indent=2))
print(json.dumps(summary, indent=2))
PY

echo "[run_one] done rc=$RC -> $OUT_DIR"
exit "$RC"
