#!/usr/bin/env python3

import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image, LaserScan
from tf2_ros import Buffer, TransformListener


SENSORS = {
    "rgb": {
        "topic": "/camera/color/image_raw",
        "message_type": Image,
        "frame_id": "camera_color_optical_frame",
        "minimum_hz": 5.0,
    },
    "depth": {
        "topic": "/camera/depth/image_raw",
        "message_type": Image,
        "frame_id": "camera_depth_optical_frame",
        "minimum_hz": 5.0,
    },
    "lidar": {
        "topic": "/scan",
        "message_type": LaserScan,
        "frame_id": "laser_frame",
        "minimum_hz": 3.0,
    },
}

TRANSFORMS = (
    ("base_link", "camera_color_optical_frame"),
    ("base_link", "camera_depth_optical_frame"),
    ("base_link", "laser_frame"),
)


class SensorHealthCheck(Node):
    def __init__(self):
        super().__init__("sensor_health_check")
        self.counts = {name: 0 for name in SENSORS}
        self.frame_ids = {name: set() for name in SENSORS}
        self.first_receive_time = {}
        self.last_receive_time = {}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=False,
        )

        self._sensor_subscriptions = []
        for name, contract in SENSORS.items():
            subscription = self.create_subscription(
                contract["message_type"],
                contract["topic"],
                lambda message, sensor_name=name: self._sensor_callback(
                    sensor_name,
                    message,
                ),
                qos_profile_sensor_data,
            )
            self._sensor_subscriptions.append(subscription)

    def _sensor_callback(self, name, message):
        now = time.monotonic()
        self.counts[name] += 1
        self.first_receive_time.setdefault(name, now)
        self.last_receive_time[name] = now
        self.frame_ids[name].add(message.header.frame_id)

    def evaluate(self):
        sensors = {}
        overall_pass = True

        for name, contract in SENSORS.items():
            count = self.counts[name]
            elapsed = (
                self.last_receive_time.get(name, 0.0)
                - self.first_receive_time.get(name, 0.0)
            )
            frequency = (
                (count - 1) / elapsed
                if count > 1 and elapsed > 0.0
                else 0.0
            )
            observed_frames = sorted(self.frame_ids[name])
            expected_frame = contract["frame_id"]
            passed = (
                count > 1
                and frequency >= contract["minimum_hz"]
                and observed_frames == [expected_frame]
            )
            overall_pass = overall_pass and passed
            sensors[name] = {
                "status": "PASS" if passed else "FAIL",
                "topic": contract["topic"],
                "messages": count,
                "rough_hz": round(frequency, 2),
                "expected_frame_id": expected_frame,
                "observed_frame_ids": observed_frames,
            }

        transforms = {}
        for parent, child in TRANSFORMS:
            key = f"{parent} -> {child}"
            try:
                transform = self.tf_buffer.lookup_transform(
                    parent,
                    child,
                    Time(),
                )
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                transforms[key] = {
                    "status": "PASS",
                    "translation": [
                        round(translation.x, 6),
                        round(translation.y, 6),
                        round(translation.z, 6),
                    ],
                    "quaternion_xyzw": [
                        round(rotation.x, 6),
                        round(rotation.y, 6),
                        round(rotation.z, 6),
                        round(rotation.w, 6),
                    ],
                }
            except Exception as error:
                overall_pass = False
                transforms[key] = {
                    "status": "FAIL",
                    "error": str(error),
                }

        return {
            "status": "PASS" if overall_pass else "FAIL",
            "sensors": sensors,
            "transforms": transforms,
        }


def main():
    parser = argparse.ArgumentParser(
        description="Bounded health check for the robot sensor and TF contract."
    )
    parser.add_argument(
        "--window",
        type=float,
        default=5.0,
        help="Observation window in seconds (minimum 2.0).",
    )
    arguments = parser.parse_args(remove_ros_args(args=sys.argv)[1:])
    window = max(2.0, arguments.window)

    rclpy.init(args=sys.argv)
    node = SensorHealthCheck()
    deadline = time.monotonic() + window
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    result = node.evaluate()
    for name, details in result["sensors"].items():
        print(
            f"{name.upper()}: {details['status']} "
            f"messages={details['messages']} "
            f"rough_hz={details['rough_hz']:.2f} "
            f"frame_ids={details['observed_frame_ids']}"
        )
    for transform, details in result["transforms"].items():
        print(f"TF {transform}: {details['status']}")
    print("BASIC_DIAGNOSTICS_V1=" + result["status"])
    print("DIAGNOSTICS_JSON=" + json.dumps(result, separators=(",", ":")))

    exit_code = 0 if result["status"] == "PASS" else 1
    node.destroy_node()
    rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
