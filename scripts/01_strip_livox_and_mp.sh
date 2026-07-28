#!/usr/bin/env bash
# Strip external livox_ros_driver deps and force dual-core MP settings.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SRC="$ROOT/src"

strip_pkg_xml() {
  local f="$1"
  [[ -f "$f" ]] || return 0
  # remove livox_ros_driver depend lines
  sed -i '/livox_ros_driver/d' "$f"
}

force_mp2_cmake() {
  local f="$1"
  [[ -f "$f" ]] || return 0
  # Replace MP_PROC_NUM autodetection blocks with fixed 2 cores
  if grep -q 'MP_PROC_NUM' "$f"; then
    python3 - "$f" <<'PY'
import re,sys
path=sys.argv[1]
text=open(path).read()
# Force after architecture check: always MP_EN + MP_PROC_NUM=2 for x86
new = re.sub(
r'if\(CMAKE_SYSTEM_PROCESSOR MATCHES.*?\nelse\(\)\n\s*add_definitions\(-DMP_PROC_NUM=1\)\nendif\(\)',
'''# LIO benchmark: force 2-thread parallel map update
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  add_definitions(-DMP_EN)
  add_definitions(-DMP_PROC_NUM=2)
  message(STATUS "LIO benchmark: MP_PROC_NUM=2")
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()''',
text, count=1, flags=re.S)
if new==text:
    # simpler: replace all MP_PROC_NUM=N with 2 and ensure MP_EN
    new=re.sub(r'add_definitions\(-DMP_PROC_NUM=\d+\)','add_definitions(-DMP_PROC_NUM=2)',text)
open(path,'w').write(new)
print('patched MP', path)
PY
  fi
  # Ensure DISABLE_LIVOX define
  if ! grep -q 'DISABLE_LIVOX' "$f"; then
    sed -i '/^project(/a add_definitions(-DDISABLE_LIVOX)' "$f"
  fi
}

# --- package.xml ---
for pkg in FAST_LIO Point-LIO VoxelMapPlus_Public R-VoxelMap; do
  strip_pkg_xml "$SRC/$pkg/package.xml"
done

# --- CMake: remove livox from find_package catkin COMPONENTS ---
for f in \
  "$SRC/FAST_LIO/CMakeLists.txt" \
  "$SRC/Point-LIO/CMakeLists.txt" \
  "$SRC/VoxelMapPlus_Public/CMakeLists.txt" \
  "$SRC/R-VoxelMap/CMakeLists.txt"
do
  [[ -f "$f" ]] || continue
  sed -i '/livox_ros_driver/d' "$f"
  force_mp2_cmake "$f"
done

# faster-lio: keep vendored msg OR disable subdirectory — disable and macro
if [[ -f "$SRC/faster-lio/CMakeLists.txt" ]]; then
  sed -i 's/add_subdirectory(thirdparty\/livox_ros_driver)/# add_subdirectory(thirdparty\/livox_ros_driver) # DISABLED/' "$SRC/faster-lio/CMakeLists.txt"
  if ! grep -q 'DISABLE_LIVOX' "$SRC/faster-lio/CMakeLists.txt"; then
    sed -i '/^project(/a add_definitions(-DDISABLE_LIVOX)' "$SRC/faster-lio/CMakeLists.txt"
  fi
  # remove gencpp dep on livox
  sed -i 's/ livox_ros_driver_gencpp//' "$SRC/faster-lio/src/CMakeLists.txt" || true
  force_mp2_cmake "$SRC/faster-lio/CMakeLists.txt"
fi

# AKF-LIO: same
if [[ -f "$SRC/AKF-LIO/CMakeLists.txt" ]]; then
  sed -i 's/add_subdirectory(thirdparty\/livox_ros_driver)/# add_subdirectory(thirdparty\/livox_ros_driver) # DISABLED/' "$SRC/AKF-LIO/CMakeLists.txt"
  if ! grep -q 'DISABLE_LIVOX' "$SRC/AKF-LIO/CMakeLists.txt"; then
    sed -i '/^project(/a add_definitions(-DDISABLE_LIVOX)' "$SRC/AKF-LIO/CMakeLists.txt"
  fi
  sed -i 's/ livox_ros_driver_gencpp//' "$SRC/AKF-LIO/src/CMakeLists.txt" || true
  force_mp2_cmake "$SRC/AKF-LIO/CMakeLists.txt"
fi

# PV-LIO: force MP=2 (already no livox in find_package)
force_mp2_cmake "$SRC/PV-LIO/CMakeLists.txt"
if [[ -f "$SRC/PV-LIO/CMakeLists.txt" ]] && ! grep -q 'DISABLE_LIVOX' "$SRC/PV-LIO/CMakeLists.txt"; then
  sed -i '/^project(/a add_definitions(-DDISABLE_LIVOX)' "$SRC/PV-LIO/CMakeLists.txt"
fi

# DLIO / Super-LIO / BIEVR: no livox usually; still force openmp threads via note
for f in \
  "$SRC/direct_lidar_inertial_odometry/CMakeLists.txt" \
  "$SRC/super_lio/CMakeLists.txt" \
  "$SRC/BIEVR-LIO/BIEVR/CMakeLists.txt"
do
  [[ -f "$f" ]] || continue
  if ! grep -q 'OMP_NUM_THREADS' "$f"; then
    # soft hint via compile definition used by some codes
    true
  fi
done

echo "CMake/package.xml livox strip + MP=2 done"
