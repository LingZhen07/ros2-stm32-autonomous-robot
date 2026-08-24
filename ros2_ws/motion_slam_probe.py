#!/usr/bin/env python3
import json
import math
import signal
import time

import rclpy
from cartographer_ros_msgs.msg import SubmapList
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


STOP_REQUESTED = False


def request_stop(_signum, _frame):
    global STOP_REQUESTED
    STOP_REQUESTED = True


def normalize_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


class MotionProbe(Node):
    def __init__(self):
        super().__init__("motion_slam_probe")
        self.started = time.monotonic()
        self.scan_count = 0
        self.first_scan_time = None
        self.last_scan_time = None
        self.map_count = 0
        self.initial_map = None
        self.latest_map = None
        self.map_change_events = 0
        self.submap_count = 0
        self.latest_submaps = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.tf_samples = 0
        self.tf_failures = 0
        self.start_pose = None
        self.current_pose = None
        self.previous_pose = None
        self.max_translation = 0.0
        self.max_abs_yaw = 0.0
        self.path_length = 0.0
        self.create_subscription(
            LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data
        )
        self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)
        self.create_subscription(SubmapList, "/submap_list", self.submap_callback, 10)

    def scan_callback(self, _message):
        now = time.monotonic()
        self.scan_count += 1
        if self.first_scan_time is None:
            self.first_scan_time = now
        self.last_scan_time = now

    @staticmethod
    def map_summary(message):
        data = message.data
        return {
            "frame_id": message.header.frame_id,
            "resolution_m": message.info.resolution,
            "width": message.info.width,
            "height": message.info.height,
            "cells": len(data),
            "unknown": sum(value < 0 for value in data),
            "free": sum(value == 0 for value in data),
            "occupied": sum(value > 0 for value in data),
            "origin_x": message.info.origin.position.x,
            "origin_y": message.info.origin.position.y,
        }

    def map_callback(self, message):
        summary = self.map_summary(message)
        self.map_count += 1
        if self.initial_map is None:
            self.initial_map = summary
        if self.latest_map is not None and summary != self.latest_map:
            self.map_change_events += 1
        self.latest_map = summary

    def submap_callback(self, message):
        self.submap_count += 1
        self.latest_submaps = {
            "frame_id": message.header.frame_id,
            "entries": len(message.submap),
        }

    def sample_tf(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                "odom", "base_link", rclpy.time.Time()
            ).transform
        except TransformException:
            self.tf_failures += 1
            return
        pose = (
            transform.translation.x,
            transform.translation.y,
            yaw_from_quaternion(transform.rotation),
        )
        self.tf_samples += 1
        if self.start_pose is None:
            self.start_pose = pose
        if self.previous_pose is not None:
            self.path_length += math.hypot(
                pose[0] - self.previous_pose[0], pose[1] - self.previous_pose[1]
            )
        self.previous_pose = pose
        self.current_pose = pose
        x0, y0, yaw0 = self.start_pose
        self.max_translation = max(
            self.max_translation, math.hypot(pose[0] - x0, pose[1] - y0)
        )
        self.max_abs_yaw = max(
            self.max_abs_yaw, abs(normalize_angle(pose[2] - yaw0))
        )

    def summary(self):
        result = {
            "ready": self.start_pose is not None
            and self.scan_count > 0
            and self.latest_map is not None,
            "elapsed_sec": round(time.monotonic() - self.started, 3),
            "scan_messages": self.scan_count,
            "scan_hz": None,
            "map_messages": self.map_count,
            "map_change_events": self.map_change_events,
            "initial_map": self.initial_map,
            "latest_map": self.latest_map,
            "submap_messages": self.submap_count,
            "submaps": self.latest_submaps,
            "tf_samples": self.tf_samples,
            "tf_failures": self.tf_failures,
            "max_translation_from_start_m": round(self.max_translation, 6),
            "max_abs_yaw_from_start_rad": round(self.max_abs_yaw, 6),
            "sampled_path_length_m": round(self.path_length, 6),
        }
        if self.scan_count > 1 and self.last_scan_time > self.first_scan_time:
            result["scan_hz"] = round(
                (self.scan_count - 1) / (self.last_scan_time - self.first_scan_time), 3
            )
        if self.start_pose is not None:
            result["start_pose"] = {
                "x": round(self.start_pose[0], 6),
                "y": round(self.start_pose[1], 6),
                "yaw_rad": round(self.start_pose[2], 6),
            }
        if self.current_pose is not None:
            result["current_pose"] = {
                "x": round(self.current_pose[0], 6),
                "y": round(self.current_pose[1], 6),
                "yaw_rad": round(self.current_pose[2], 6),
            }
        return result

    def write_summary(self):
        with open("/tmp/slam_motion_summary.json", "w", encoding="utf-8") as output:
            json.dump(self.summary(), output, sort_keys=True, indent=2)
            output.write("\n")


def main():
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    rclpy.init()
    node = MotionProbe()
    deadline = time.monotonic() + 900.0
    next_tf_sample = time.monotonic()
    next_write = time.monotonic()
    while not STOP_REQUESTED and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.monotonic()
        if now >= next_tf_sample:
            node.sample_tf()
            next_tf_sample = now + 0.1
        if now >= next_write:
            node.write_summary()
            next_write = now + 1.0
    node.write_summary()
    print(json.dumps(node.summary(), sort_keys=True))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
