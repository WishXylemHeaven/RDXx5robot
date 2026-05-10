#!/usr/bin/env python3
import argparse
import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class TfWaiter(Node):
    def __init__(self):
        super().__init__('digua_wait_for_tf')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def wait(self, target_frame, source_frame, timeout_sec):
        start = time.time()
        last_error = ""

        while rclpy.ok() and time.time() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                self.buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rclpy.time.Time()
                )
                self.get_logger().info(f'TF ready: {target_frame} -> {source_frame}')
                return True
            except Exception as e:
                last_error = str(e)

        self.get_logger().error(
            f'Timeout waiting for TF: {target_frame} -> {source_frame}. Last error: {last_error}'
        )
        return False


def main():
    parser = argparse.ArgumentParser(description='Wait for a TF transform.')
    parser.add_argument('target_frame', type=str, help='Target frame, e.g. map.')
    parser.add_argument('source_frame', type=str, help='Source frame, e.g. odom.')
    parser.add_argument('--timeout', type=float, default=30.0, help='Timeout seconds.')
    args = parser.parse_args()

    rclpy.init()
    node = TfWaiter()

    try:
        ok = node.wait(args.target_frame, args.source_frame, args.timeout)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
