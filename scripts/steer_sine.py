#!/usr/bin/env python3
"""
Publish a sine wave to a ROS 2 steering topic (std_msgs/msg/Float32).

The value is in radians (angle mode), so it goes through the node's full
angle → degree → ratio conversion pipeline.

Usage:
    python3 steer_sine.py
    python3 steer_sine.py --amplitude 0.25 --frequency 0.1
    python3 steer_sine.py --topic /leader/steering_cmd --amplitude 0.5 --rate 50
    python3 steer_sine.py --help

Safety notes:
  - Default amplitude (0.3 rad ≈ 17.2°) is within the node's default
    steering_clip of 20°.  The node will clamp regardless.
  - On Ctrl+C the script publishes 0.0 rad before exiting so the
    steering returns to center.
"""

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class SteeringSinePublisher(Node):
    def __init__(self, topic: str, amplitude: float, frequency: float, rate_hz: float):
        super().__init__("steering_sine_publisher")

        self._amplitude = amplitude
        self._frequency = frequency
        self._start_time = self.get_clock().now()

        self._pub = self.create_publisher(Float32, topic, 10)
        self._timer = self.create_timer(1.0 / rate_hz, self._timer_cb)

        self.get_logger().info(
            f"Steering sine wave publisher started\n"
            f"  topic     : {topic}\n"
            f"  amplitude : {amplitude:.4f} rad  ({math.degrees(amplitude):.2f} deg)\n"
            f"  frequency : {frequency:.3f} Hz  (period = {1.0/frequency:.1f} s)\n"
            f"  pub rate  : {rate_hz:.1f} Hz"
        )

    def _timer_cb(self) -> None:
        elapsed = (self.get_clock().now() - self._start_time).nanoseconds * 1e-9
        value = self._amplitude * math.sin(2.0 * math.pi * self._frequency * elapsed)

        msg = Float32()
        msg.data = float(value)
        self._pub.publish(msg)

        self.get_logger().info(
            f"steering: {value:+.4f} rad  ({math.degrees(value):+.2f} deg)",
            throttle_duration_sec=0.5,
        )

    def publish_zero(self) -> None:
        """Publish a center (zero) command — call before shutdown."""
        msg = Float32()
        msg.data = 0.0
        self._pub.publish(msg)
        self.get_logger().info("Shutdown: published 0.0 rad (center)")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Publish a sine wave to a ROS 2 steering topic (Float32, radians).",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--topic",
        default="/follower/steering_cmd",
        help="ROS 2 topic to publish on",
    )
    parser.add_argument(
        "--amplitude",
        type=float,
        default=0.3,
        help="Sine wave amplitude in radians (0.3 rad ≈ 17.2°, within the default 20° clip)",
    )
    parser.add_argument(
        "--frequency",
        type=float,
        default=0.2,
        help="Sine wave frequency in Hz (0.2 Hz = one full sweep every 5 s)",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=20.0,
        help="Publish rate in Hz",
    )
    # Strip ROS 2 args before passing to argparse
    argv = argv or sys.argv[1:]
    argv = [a for a in argv if not a.startswith("__") and not a.startswith("--ros-args")]
    return parser.parse_args(argv)


def main():
    args = parse_args()

    if args.amplitude <= 0:
        print("ERROR: --amplitude must be > 0", file=sys.stderr)
        sys.exit(1)
    if args.frequency <= 0:
        print("ERROR: --frequency must be > 0", file=sys.stderr)
        sys.exit(1)
    if args.rate <= 0:
        print("ERROR: --rate must be > 0", file=sys.stderr)
        sys.exit(1)

    rclpy.init()
    node = SteeringSinePublisher(args.topic, args.amplitude, args.frequency, args.rate)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_zero()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
