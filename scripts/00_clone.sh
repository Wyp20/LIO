#!/usr/bin/env bash
# Clone 11 LIO algorithms into LIO/src (no livox_ros_driver).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SRC="$ROOT/src"
mkdir -p "$SRC" "$ROOT/third_party"
cd "$SRC"

clone_or_update() {
  local url="$1"
  local dest="$2"
  local branch="${3:-}"
  if [[ -d "$dest/.git" ]]; then
    echo "[skip] $dest already exists"
    return 0
  fi
  if [[ -n "$branch" ]]; then
    git clone --depth 1 -b "$branch" "$url" "$dest"
  else
    git clone --depth 1 "$url" "$dest"
  fi
}

echo "=== Cloning into $SRC ==="

clone_or_update https://github.com/hku-mars/FAST_LIO.git FAST_LIO
clone_or_update https://github.com/gaoxiang12/faster-lio.git faster-lio
clone_or_update https://github.com/vectr-ucla/direct_lidar_inertial_odometry.git direct_lidar_inertial_odometry
clone_or_update https://github.com/hku-mars/Point-LIO.git Point-LIO
clone_or_update https://github.com/uestc-icsp/VoxelMapPlus_Public.git VoxelMapPlus_Public
clone_or_update https://github.com/xpxie/AKF-LIO.git AKF-LIO
clone_or_update https://github.com/NKU-MobFly-Robotics/R-VoxelMap.git R-VoxelMap
clone_or_update https://github.com/PRBonn/rko_lio.git "$ROOT/third_party/rko_lio"

# PV-LIO: try public, fallback to local copy
if [[ ! -d PV-LIO/.git && ! -d PV-LIO/package.xml ]]; then
  if git clone --depth 1 https://github.com/hviktortsoi/PV_LIO.git PV-LIO 2>/dev/null; then
    echo "[ok] PV-LIO from github"
  elif [[ -d /home/wyp/Project_lidar_navigation/ws_LIO/src/PV-LIO ]]; then
    echo "[fallback] copy PV-LIO from local ws_LIO"
    cp -a /home/wyp/Project_lidar_navigation/ws_LIO/src/PV-LIO "$SRC/PV-LIO"
  else
    echo "[error] PV-LIO unavailable" >&2
    exit 1
  fi
else
  echo "[skip] PV-LIO already present"
fi

# Super-LIO: ros1 branch is a workspace; flatten packages into src/
if [[ ! -d super_lio ]]; then
  tmp="$ROOT/third_party/Super-LIO-tmp"
  rm -rf "$tmp"
  git clone --depth 1 -b ros1 https://github.com/Liansheng-Wang/Super-LIO.git "$tmp"
  if [[ -d "$tmp/src/super_lio" ]]; then
    cp -a "$tmp/src/super_lio" "$SRC/super_lio"
    [[ -d "$tmp/src/basic" ]] && cp -a "$tmp/src/basic" "$SRC/basic"
  else
    # already flat?
    cp -a "$tmp" "$SRC/super_lio"
  fi
  echo "[ok] Super-LIO flattened"
else
  echo "[skip] super_lio already present"
fi

# BIEVR-LIO: flatten packages
if [[ ! -d bievr_lio && ! -d BIEVR ]]; then
  tmp="$ROOT/third_party/BIEVR-LIO-tmp"
  rm -rf "$tmp"
  git clone --depth 1 https://github.com/ethz-asl/BIEVR-LIO.git "$tmp"
  # Prefer interfaces/ros1 layout
  if [[ -d "$tmp/BIEVR" ]]; then
    cp -a "$tmp/BIEVR" "$SRC/BIEVR"
  fi
  if [[ -d "$tmp/interfaces/ros1" ]]; then
    cp -a "$tmp/interfaces/ros1" "$SRC/bievr_lio_ros"
  fi
  if [[ -d "$tmp/interfaces/ros_common" ]]; then
    cp -a "$tmp/interfaces/ros_common" "$SRC/bievr_ros_common"
  fi
  # Keep configs for convenience
  mkdir -p "$ROOT/configs/bievr_upstream"
  [[ -d "$tmp/config" ]] && cp -a "$tmp/config/." "$ROOT/configs/bievr_upstream/"
  echo "[ok] BIEVR-LIO flattened"
else
  echo "[skip] BIEVR already present"
fi

echo "=== Clone done ==="
ls -la "$SRC"
