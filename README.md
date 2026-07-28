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
