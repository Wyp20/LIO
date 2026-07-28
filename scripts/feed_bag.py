#!/usr/bin/env python3
"""Publish /lidar_points + /fvs/imu_raw from a bag via rosbag API (not rosbag play)."""
from __future__ import annotations

import argparse
import sys
import time

import rosbag
import rospy
from sensor_msgs.msg import Imu, PointCloud2


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--lid-topic", default="/lidar_points")
    ap.add_argument("--imu-topic", default="/fvs/imu_raw")
    ap.add_argument("--max-sec", type=float, default=0.0, help="Stop after this much bag time (0=all)")
    ap.add_argument("--rate", type=float, default=0.0, help="Playback rate; 0 = as-fast-as-possible")
    ap.add_argument("--warmup-sec", type=float, default=1.0)
    args = ap.parse_args()

    rospy.init_node("lio_feed_bag", anonymous=True)
    pub_lid = rospy.Publisher(args.lid_topic, PointCloud2, queue_size=10)
    pub_imu = rospy.Publisher(args.imu_topic, Imu, queue_size=200)

    # Wait for subscribers so early frames are not dropped.
    t0 = time.time()
    while time.time() - t0 < args.warmup_sec and not rospy.is_shutdown():
        if pub_lid.get_num_connections() > 0 and pub_imu.get_num_connections() > 0:
            break
        time.sleep(0.05)
    time.sleep(0.2)

    topics = [args.lid_topic, args.imu_topic]
    bag = rosbag.Bag(args.bag, "r")
    n_lid = n_imu = 0
    t_start = None
    wall0 = None
    bag_t0 = None

    try:
        for topic, msg, t in bag.read_messages(topics=topics):
            if rospy.is_shutdown():
                break
            ts = t.to_sec()
            if t_start is None:
                t_start = ts
                bag_t0 = ts
                wall0 = time.time()
            if args.max_sec > 0 and (ts - t_start) > args.max_sec:
                break
            if args.rate > 0 and bag_t0 is not None and wall0 is not None:
                target = wall0 + (ts - bag_t0) / args.rate
                delay = target - time.time()
                if delay > 0:
                    time.sleep(delay)

            if topic == args.lid_topic:
                pub_lid.publish(msg)
                n_lid += 1
            else:
                pub_imu.publish(msg)
                n_imu += 1
    finally:
        bag.close()

    rospy.loginfo("feed_bag done: lidar=%d imu=%d", n_lid, n_imu)
    # Give algorithms a moment to flush the last frames.
    time.sleep(1.0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
