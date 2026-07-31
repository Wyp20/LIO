# LIO Hesai Online Evaluation

ROS Noetic workspace for running multiple LIO algorithms online on Hesai bags
(lidar + IMU topics), with RViz enabled by default.

## Environment

```bash
export LIO_WS=/path/to/LIO   # this workspace root
source /opt/ros/noetic/setup.bash
source $LIO_WS/devel/setup.bash
```

Example bag:

```bash
BAG=/path/to/your.bag
```

## Single-thread offline evaluation

Algorithms are forced to **1 thread** (`MP_PROC_NUM=1`, `OMP/TBB_NUM_THREADS=1`,
`taskset -c 0`).

```bash
# Rebuild after thread / feeder changes
cd $LIO_WS && catkin_make -DCMAKE_BUILD_TYPE=Release -j$(nproc)

# Full offline eval (CPU%, mem%, frame ms, dense PCL ASCII clouds)
$LIO_WS/scripts/eval_hesai_1thread.sh "$BAG"

# Or one algo:
$LIO_WS/scripts/run_one.sh --algo fast_lio --bag "$BAG"
```

Results land in `results/eval_1thread/<bag>_<window>_1t/<algo>/`:
- `summary.json` — CPU/mem/frame timing
- `cloud_merged.txt` — dense published clouds (PCL ASCII, no downsample), under
  `/path/to/lio_eval_clouds/<tag>/<algo>/` by default (C++ `save_clouds_node`)
- `resources.csv`, `timing.json`, `run.log`
- Sorted CSVs: `summary_by_rss_peak.csv`, `summary_by_frame_max.csv`, `summary_by_frame_mean.csv`

### Eval summary (single-thread)

Sorted by mean frame time. Peak RSS = process peak memory.

| algo | RSS peak (MB) | avg ms | max ms | frames |
|------|---------------|--------|--------|--------|
| voxel_map_plus | 205.6 | 3.00 | 3.74 | 1384 |
| super_lio | 186.2 | 7.03 | 11.49 | 1381 |
| pv_lio | 327.0 | 7.95 | 11.68 | 1383 |
| rvoxelmap | 367.6 | 9.47 | 16.08 | 1384 |
| point_lio | 208.1 | 12.70 | 19.39 | 1384 |
| fast_lio | 315.7 | 13.25 | 32.91 | 1383 |
| dlio | 215.3 | 13.55 | 32.73 | 1365 |
| faster_lio | 283.4 | 14.63 | 36.66 | 1396 |
| bievr | 449.8 | 22.43 | 83.92 | 1394 |
| akf_lio | 245.1 | 106.25 | 148.96 | 1384 |

## Online run (algorithm + RViz)

Start **one** algorithm in a terminal (do not pass `bag_path`; leave it empty for topic mode).
In another terminal, play the bag.

```bash
rosbag play --clock "$BAG" /lidar_topic /imu_topic
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
