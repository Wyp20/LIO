#!/usr/bin/env bash
# Clone 11 LIO algorithms into LIO/src (no livox_ros_driver).
# After clone, nested .git is removed so only the outer LIO monorepo remains.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SRC="$ROOT/src"
mkdir -p "$SRC" "$ROOT/third_party"
cd "$SRC"

strip_nested_git() {
  local dest="$1"
  if [[ -e "$dest/.git" ]]; then
    rm -rf "$dest/.git"
    echo "[strip] removed nested .git from $dest"
  fi
  # Drop leftover submodule pointers / modules metadata inside the tree
  find "$dest" -name ".git" -exec rm -rf {} + 2>/dev/null || true
}

clone_or_update() {
  local url="$1"
  local dest="$2"
  local branch="${3:-}"
  if [[ -d "$dest" && ( -f "$dest/package.xml" || -f "$dest/CMakeLists.txt" || -d "$dest/src" ) ]]; then
    echo "[skip] $dest already present"
    strip_nested_git "$dest"
    return 0
  fi
  if [[ -n "$branch" ]]; then
    git clone --depth 1 -b "$branch" "$url" "$dest"
  else
    git clone --depth 1 "$url" "$dest"
  fi
  strip_nested_git "$dest"
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
if [[ ! -d PV-LIO/package.xml ]]; then
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
[[ -d PV-LIO ]] && strip_nested_git PV-LIO

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
  rm -rf "$tmp"
  echo "[ok] Super-LIO flattened"
else
  echo "[skip] super_lio already present"
fi
[[ -d super_lio ]] && strip_nested_git super_lio
[[ -d basic ]] && strip_nested_git basic
# Keep a non-git snapshot of Super-LIO upstream tree if present
[[ -d "$ROOT/third_party/Super-LIO" ]] && strip_nested_git "$ROOT/third_party/Super-LIO"

# BIEVR-LIO: keep upstream layout (BIEVR/ + interfaces/{ros1,ros2,ros_common})
if [[ ! -d BIEVR-LIO ]]; then
  git clone --depth 1 https://github.com/ethz-asl/BIEVR-LIO.git "$SRC/BIEVR-LIO"
  # ROS1 workspace: skip ament ROS2 package
  touch "$SRC/BIEVR-LIO/interfaces/ros2/CATKIN_IGNORE"
  # Convenience copy of upstream configs (Hesai overrides live in configs/hesai/)
  mkdir -p "$ROOT/configs/bievr_upstream"
  [[ -d "$SRC/BIEVR-LIO/config" ]] && cp -a "$SRC/BIEVR-LIO/config/." "$ROOT/configs/bievr_upstream/"
  echo "[ok] BIEVR-LIO (original layout)"
else
  echo "[skip] BIEVR-LIO already present"
fi
[[ -d BIEVR-LIO ]] && strip_nested_git BIEVR-LIO

echo "=== Clone done (nested .git stripped; outer LIO repo only) ==="
ls -la "$SRC"
