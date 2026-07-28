#!/usr/bin/env bash
# Batch runner stub: iterate algos x bags (filtered list).
set -euo pipefail
LIO_WS="${LIO_WS:-/home/wyp/Project_lidar_navigation/LIO}"
export LIO_WS
BAG_LIST="${1:-}"
ALGOS="${ALGOS:-faster_lio fast_lio point_lio pv_lio akf_lio voxel_map_plus rvoxelmap super_lio dlio bievr}"
MAX_SEC="${MAX_SEC:-0}"

if [[ -z "$BAG_LIST" || ! -f "$BAG_LIST" ]]; then
  echo "Usage: $0 <bags.txt>   # one bag path per line"
  exit 1
fi

while IFS= read -r bag; do
  [[ -z "$bag" || "$bag" =~ ^# ]] && continue
  stem="$(basename "$bag" .bag)"
  for a in $ALGOS; do
    echo "==== $a :: $stem ===="
    "${LIO_WS}/scripts/run_one.sh" --algo "$a" --bag "$bag" --max-sec "$MAX_SEC" \
      --out "${LIO_WS}/results/${a}/${stem}" || echo "FAIL $a $stem"
  done
done < "$BAG_LIST"
