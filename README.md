# LIO Hesai Online Evaluation

ROS Noetic workspace for running multiple LIO algorithms online on Hesai bags
(`/lidar_points` + `/fvs/imu_raw`), with RViz enabled by default.

## Environment

```bash
export LIO_WS=/home/wyp/Project_lidar_navigation/LIO
source /opt/ros/noetic/setup.bash
source $LIO_WS/devel/setup.bash
```

Example bag:

```bash
BAG=/media/wyp/娱乐/qiao_lidar_data/auto_record_20260716_075831.bag
```

## Single-thread offline evaluation

Algorithms are forced to **1 thread** (`MP_PROC_NUM=1`, `OMP/TBB_NUM_THREADS=1`,
`taskset -c 0`). Eval window defaults to bag-relative **15s–155s** (start=15, duration=140).

```bash
# Rebuild after thread / feeder changes
cd $LIO_WS && catkin_make -DCMAKE_BUILD_TYPE=Release -j$(nproc)

# Full offline eval (CPU%, mem%, frame ms, dense PCL ASCII clouds)
$LIO_WS/scripts/eval_hesai_1thread.sh "$BAG"

# Or one algo:
$LIO_WS/scripts/run_one.sh --algo fast_lio --bag "$BAG" --start-sec 15 --max-sec 140
```

Results land in `results/eval_1thread/<bag>_<window>_1t/<algo>/`:
- `summary.json` — CPU/mem/frame timing
- `cloud_merged.txt` — dense published clouds (PCL ASCII, no downsample), under
  `/media/wyp/娱乐/lio_eval_clouds/<tag>/<algo>/` by default (C++ `save_clouds_node`)
- `resources.csv`, `timing.json`, `run.log`
- Sorted CSVs: `summary_by_rss_peak.csv`, `summary_by_frame_max.csv`, `summary_by_frame_mean.csv`

### Eval summary (single-thread, bag 15–155s)

`pv_lio` / `akf_lio` from chrono re-run; others from `*_1t_ft`. Peak RSS = process peak memory.

**By peak memory (RSS MB ↑)**

| # | algo | RSS peak (MB) | avg ms | max ms | frames |
|---|------|---------------|--------|--------|--------|
| 1 | super_lio | 186.2 | 7.03 | 11.49 | 1381 |
| 2 | voxel_map_plus | 205.6 | 3.00 | 3.74 | 1384 |
| 3 | point_lio | 208.1 | 12.70 | 19.39 | 1384 |
| 4 | dlio | 215.3 | 13.55 | 32.73 | 1365 |
| 5 | akf_lio | 245.1 | 106.25 | 148.96 | 1384 |
| 6 | faster_lio | 283.4 | 14.63 | 36.66 | 1396 |
| 7 | fast_lio | 315.7 | 13.25 | 32.91 | 1383 |
| 8 | pv_lio | 327.0 | 7.95 | 11.68 | 1383 |
| 9 | rvoxelmap | 367.6 | 9.47 | 16.08 | 1384 |
| 10 | bievr | 449.8 | 22.43 | 83.92 | 1394 |

**By max frame time (ms ↑)**

| # | algo | RSS peak (MB) | avg ms | max ms | frames |
|---|------|---------------|--------|--------|--------|
| 1 | voxel_map_plus | 205.6 | 3.00 | 3.74 | 1384 |
| 2 | super_lio | 186.2 | 7.03 | 11.49 | 1381 |
| 3 | pv_lio | 327.0 | 7.95 | 11.68 | 1383 |
| 4 | rvoxelmap | 367.6 | 9.47 | 16.08 | 1384 |
| 5 | point_lio | 208.1 | 12.70 | 19.39 | 1384 |
| 6 | dlio | 215.3 | 13.55 | 32.73 | 1365 |
| 7 | fast_lio | 315.7 | 13.25 | 32.91 | 1383 |
| 8 | faster_lio | 283.4 | 14.63 | 36.66 | 1396 |
| 9 | bievr | 449.8 | 22.43 | 83.92 | 1394 |
| 10 | akf_lio | 245.1 | 106.25 | 148.96 | 1384 |

**By mean frame time (ms ↑)**

| # | algo | RSS peak (MB) | avg ms | max ms | frames |
|---|------|---------------|--------|--------|--------|
| 1 | voxel_map_plus | 205.6 | 3.00 | 3.74 | 1384 |
| 2 | super_lio | 186.2 | 7.03 | 11.49 | 1381 |
| 3 | pv_lio | 327.0 | 7.95 | 11.68 | 1383 |
| 4 | rvoxelmap | 367.6 | 9.47 | 16.08 | 1384 |
| 5 | point_lio | 208.1 | 12.70 | 19.39 | 1384 |
| 6 | fast_lio | 315.7 | 13.25 | 32.91 | 1383 |
| 7 | dlio | 215.3 | 13.55 | 32.73 | 1365 |
| 8 | faster_lio | 283.4 | 14.63 | 36.66 | 1396 |
| 9 | bievr | 449.8 | 22.43 | 83.92 | 1394 |
| 10 | akf_lio | 245.1 | 106.25 | 148.96 | 1384 |

## Online run (algorithm + RViz)

Start **one** algorithm in a terminal (do not pass `bag_path`; leave it empty for topic mode).
In another terminal, play the bag.

Suggested play window (15s–150s relative to bag start):

```bash
rosbag play --clock -s 15 -u 135 "$BAG" /lidar_points /fvs/imu_raw
```

Optional:

```bash
rosparam set /use_sim_time true
```

Disable visualization with `rviz:=false` if needed.

### Algorithms

```bash
roslaunch $LIO_WS/launch/hesai/faster_lio_online.launch
roslaunch $LIO_WS/launch/hesai/fast_lio.launch
roslaunch $LIO_WS/launch/hesai/point_lio.launch
roslaunch $LIO_WS/launch/hesai/pv_lio.launch
roslaunch $LIO_WS/launch/hesai/akf_lio.launch
roslaunch $LIO_WS/launch/hesai/voxel_map_plus.launch
roslaunch $LIO_WS/launch/hesai/rvoxelmap.launch
roslaunch $LIO_WS/launch/hesai/super_lio.launch
roslaunch $LIO_WS/launch/hesai/dlio.launch
```

### BIEVR (needs Ceres shared library path)

BIEVR links against a locally built `libceres.so.4`. Export the library path
**before** launching, otherwise you get
`error while loading shared libraries: libceres.so.4`.

```bash
export LD_LIBRARY_PATH="${HOME}/.local/lib:${HOME}/.cache/ceres-build/build/lib:${LD_LIBRARY_PATH}"

# Online (subscribe topics + rosbag play)
roslaunch $LIO_WS/launch/hesai/bievr_online.launch

# Or offline (node reads the bag itself)
# roslaunch $LIO_WS/launch/hesai/bievr.launch rosbag:=$BAG
```

## Notes

- Leave `bag_path` empty for online mode. A non-empty `bag_path` makes the node
  read the rosbag internally (offline) and exit when the bag ends.
- Configs live under `configs/hesai/`; launches under `launch/hesai/`.
- Headless smoke/benchmark scripts force `rviz:=false` via `scripts/run_one.sh`.
