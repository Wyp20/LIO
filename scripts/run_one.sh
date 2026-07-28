#!/usr/bin/env bash
# Run one (algo, bag) pair under dual-core limits and record resources.
set -euo pipefail

LIO_WS="${LIO_WS:-/home/wyp/Project_lidar_navigation/LIO}"
export LIO_WS
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1091
source "${LIO_WS}/devel/setup.bash"

ALGO=""
BAG=""
MAX_SEC=0
OUT_DIR=""
CPUS="${CPUS:-0,1}"

usage() {
  cat <<EOF
Usage: $0 --algo <name> --bag <path> [--max-sec N] [--out DIR] [--cpus 0,1]

Algos:
  fast_lio faster_lio point_lio pv_lio akf_lio voxel_map_plus
  rvoxelmap super_lio dlio bievr rko_lio
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --algo) ALGO="$2"; shift 2 ;;
    --bag) BAG="$2"; shift 2 ;;
    --max-sec) MAX_SEC="$2"; shift 2 ;;
    --out) OUT_DIR="$2"; shift 2 ;;
    --cpus) CPUS="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown arg: $1"; usage; exit 1 ;;
  esac
done

[[ -n "$ALGO" && -n "$BAG" ]] || { usage; exit 1; }
[[ -f "$BAG" ]] || { echo "bag not found: $BAG"; exit 1; }

BAG_STEM="$(basename "$BAG" .bag)"
OUT_DIR="${OUT_DIR:-${LIO_WS}/results/${ALGO}/${BAG_STEM}}"
mkdir -p "$OUT_DIR"
STOP_FILE="${OUT_DIR}/.stop_monitor"
rm -f "$STOP_FILE"

export OMP_NUM_THREADS=2
export MKL_NUM_THREADS=2
export OPENBLAS_NUM_THREADS=2
export NUMEXPR_NUM_THREADS=2
export TBB_NUM_THREADS=2
export VECLIB_MAXIMUM_THREADS=2

LOG="${OUT_DIR}/run.log"
RES_CSV="${OUT_DIR}/resources.csv"
RES_JSON="${OUT_DIR}/resources.json"
TIMING_JSON="${OUT_DIR}/timing.json"
META_JSON="${OUT_DIR}/meta.json"
: > "$LOG"

echo "[run_one] algo=$ALGO bag=$BAG max_sec=$MAX_SEC out=$OUT_DIR cpus=$CPUS"

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

start_monitor() {
  local pid="$1"
  python3 "${LIO_WS}/scripts/monitor_resources.py" \
    --pid "$pid" --out "$RES_CSV" --summary "$RES_JSON" \
    --interval 0.5 --stop-file "$STOP_FILE" >"${OUT_DIR}/monitor.log" 2>&1 &
  echo $! > "${OUT_DIR}/monitor.pid"
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
  touch "$STOP_FILE" || true
  if [[ -n "${ALGO_PGID:-}" ]]; then
    kill -TERM -"$ALGO_PGID" 2>/dev/null || true
  fi
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill "$LAUNCH_PID" 2>/dev/null || true
  fi
  # Do not kill shared roscore by default
}
trap cleanup EXIT

python3 - <<PY
import json, time
from pathlib import Path
meta = {
  "algo": "$ALGO",
  "bag": "$BAG",
  "max_sec": float("$MAX_SEC"),
  "cpus": "$CPUS",
  "omp_num_threads": 2,
  "started_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
  "config_dir": "${LIO_WS}/configs/hesai",
}
Path("$META_JSON").write_text(json.dumps(meta, indent=2))
PY

run_feed() {
  # Online nodes cannot keep up with as-fast-as-possible flood; default 1x realtime.
  local rate="${FEED_RATE:-1.0}"
  local feed_args=(--bag "$BAG" --lid-topic /lidar_points --imu-topic /fvs/imu_raw --rate "$rate" --warmup-sec 3)
  if [[ "$MAX_SEC" != "0" && "$MAX_SEC" != "0.0" ]]; then
    feed_args+=(--max-sec "$MAX_SEC")
  fi
  echo "[run_one] feeding bag rate=$rate" | tee -a "$LOG"
  taskset -c "$CPUS" python3 "${LIO_WS}/scripts/feed_bag.py" "${feed_args[@]}" \
    >>"$LOG" 2>&1
}

wait_for_node() {
  local pattern="$1"
  local deadline=$((SECONDS + 30))
  while (( SECONDS < deadline )); do
    if rosnode list 2>/dev/null | grep -q "$pattern"; then
      echo "[run_one] node ready: $pattern" | tee -a "$LOG"
      return 0
    fi
    sleep 0.2
  done
  echo "[run_one] WARNING: node not seen: $pattern" | tee -a "$LOG"
  return 1
}

case "$ALGO" in
  faster_lio)
    # Native offline reader (gflags --bag_file)
    CFG="${LIO_WS}/configs/hesai/faster_lio.yaml"
    BIN="${LIO_WS}/devel/lib/faster_lio/run_mapping_offline"
    [[ -x "$BIN" ]] || BIN="$(find "${LIO_WS}/devel" -name run_mapping_offline -type f | head -1)"
    [[ -x "$BIN" ]] || { echo "run_mapping_offline not found"; exit 1; }
    TRAJ="${OUT_DIR}/traj.txt"
    TLOG="${OUT_DIR}/time.log"
    set +e
    taskset -c "$CPUS" "$BIN" \
      --config_file="$CFG" \
      --bag_file="$BAG" \
      --traj_log_file="$TRAJ" \
      --time_log_file="$TLOG" \
      >>"$LOG" 2>&1 &
    ALGO_PID=$!
    start_monitor "$ALGO_PID"
    wait "$ALGO_PID"
    RC=$?
    set -e
    stop_monitor
    ;;

  bievr)
    ensure_roscore
    export LD_LIBRARY_PATH="${HOME}/.local/lib:${HOME}/.cache/ceres-build/build/lib:${LD_LIBRARY_PATH:-}"
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
    # Native offline via bag_path param (node reads rosbag internally)
    ensure_roscore
    LAUNCH_FILE="${LIO_WS}/launch/hesai/${ALGO}.launch"
    set +e
    taskset -c "$CPUS" roslaunch "$LAUNCH_FILE" bag_path:="$BAG" rviz:=false >>"$LOG" 2>&1 &
    LAUNCH_PID=$!
    start_monitor "$LAUNCH_PID"
    wait "$LAUNCH_PID"
    RC=$?
    set -e
    stop_monitor
    if grep -qE 'Segmentation fault|Aborted \(core dumped\)|Failed to open bag|exit code -11|process has died' "$LOG"; then
      RC=1
    elif grep -qE 'Offline bag opened|OFFLINE|offline bag' "$LOG"; then
      # finished (roslaunch may return non-zero on required exit)
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
p = Path("$META_JSON")
meta = json.loads(p.read_text()) if p.exists() else {}
meta["finished_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")
meta["exit_code"] = int("$RC")
meta["out_dir"] = "$OUT_DIR"
p.write_text(json.dumps(meta, indent=2))
print(json.dumps(meta, indent=2))
PY

echo "[run_one] done rc=$RC -> $OUT_DIR"
exit "$RC"
