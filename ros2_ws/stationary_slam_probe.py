#!/usr/bin/env python3
import json
import math
import time

import rclpy
from cartographer_ros_msgs.msg import SubmapList
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def normalize_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


class StationaryProbe(Node):
    def __init__(self):
        super().__init__("stationary_slam_probe")
        self.scan_receipts = []
        self.map_count = 0
        self.last_map = None
        self.submap_count = 0
        self.last_submaps = None
        self.tf_samples = []
        self.tf_failures = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.create_subscription(
            LaserScan, "/scan", self._scan_callback, qos_profile_sensor_data
        )
        self.create_subscription(OccupancyGrid, "/map", self._map_callback, 10)
        self.create_subscription(SubmapList, "/submap_list", self._submap_callback, 10)

    def _scan_callback(self, _message):
        self.scan_receipts.append(time.monotonic())

    def _map_callback(self, message):
        self.map_count += 1
        self.last_map = message

    def _submap_callback(self, message):
        self.submap_count += 1
        self.last_submaps = message

    def sample_tf(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                "odom", "base_link", rclpy.time.Time()
            ).transform
            self.tf_samples.append(
                (
                    transform.translation.x,
                    transform.translation.y,
                    yaw_from_quaternion(transform.rotation),
                )
            )
        except TransformException:
            self.tf_failures += 1


def main():
    duration_sec = 30.0
    rclpy.init()
    node = StationaryProbe()
    start = time.monotonic()
    next_tf_sample = start
    while time.monotonic() - start < duration_sec:
        rclpy.spin_once(node, timeout_sec=0.1)
        now = time.monotonic()
        if now >= next_tf_sample:
            node.sample_tf()
            next_tf_sample = now + 0.5

    result = {
        "duration_sec": round(time.monotonic() - start, 3),
        "scan_messages": len(node.scan_receipts),
        "scan_hz": None,
        "map_messages": node.map_count,
        "submap_messages": node.submap_count,
        "tf_samples": len(node.tf_samples),
        "tf_failures": node.tf_failures,
    }
    if len(node.scan_receipts) > 1:
        result["scan_hz"] = round(
            (len(node.scan_receipts) - 1)
            / (node.scan_receipts[-1] - node.scan_receipts[0]),
            3,
        )
    if node.tf_samples:
        x0, y0, yaw0 = node.tf_samples[0]
        x1, y1, yaw1 = node.tf_samples[-1]
        result["odom_to_base_start"] = {
            "x": round(x0, 6),
            "y": round(y0, 6),
            "yaw_rad": round(yaw0, 6),
        }
        result["odom_to_base_end"] = {
            "x": round(x1, 6),
            "y": round(y1, 6),
            "yaw_rad": round(yaw1, 6),
        }
        result["stationary_net_translation_m"] = round(math.hypot(x1 - x0, y1 - y0), 6)
        result["stationary_net_yaw_rad"] = round(normalize_angle(yaw1 - yaw0), 6)
        result["stationary_max_translation_from_start_m"] = round(
            max(math.hypot(x - x0, y - y0) for x, y, _yaw in node.tf_samples), 6
        )
        result["stationary_max_yaw_from_start_rad"] = round(
            max(abs(normalize_angle(yaw - yaw0)) for _x, _y, yaw in node.tf_samples), 6
        )
    if node.last_map is not None:
        data = node.last_map.data
        result["map"] = {
            "frame_id": node.last_map.header.frame_id,
            "resolution_m": node.last_map.info.resolution,
            "width": node.last_map.info.width,
            "height": node.last_map.info.height,
            "cells": len(data),
            "unknown": sum(value < 0 for value in data),
            "free": sum(value == 0 for value in data),
            "occupied": sum(value > 0 for value in data),
        }
    if node.last_submaps is not None:
        result["submaps"] = {
            "frame_id": node.last_submaps.header.frame_id,
            "entries": len(node.last_submaps.submap),
        }
    print(json.dumps(result, sort_keys=True, indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
