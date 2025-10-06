from __future__ import annotations

from typing import Any

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class RicohThetaPublisher(Node):
    """Capture frames from a RICOH THETA stream and republish them as ROS images."""

    def __init__(self) -> None:
        super().__init__("ricoh_theta_publisher")

        pipeline = "thetauvcsrc ! decodebin ! videoconvert ! video/x-raw,format=BGR ! appsink sync=false"
        self.publisher_ = self.create_publisher(Image, "camera360", 10)
        self.status_pub = self.create_publisher(String, "camera360/status", 10)
        self.frame_id = "camera_ricoh"
        self.get_logger().info(f"Using pipeline: {pipeline}")
        self.frame_count = 0

        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera")
            self._publish_status("Camera failed to open")
            return

        self.get_logger().info("Camera opened, starting stream...")
        self.timer = self.create_timer(0.033, self.timer_callback)  # ~30 FPS

    def timer_callback(self) -> None:
        if not self.cap:
            self.get_logger().error("Video capture is uninitialized")
            return

        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warning("Failed to read frame")
            self._publish_status("Frame read failed")
            return

        self.frame_count += 1
        if self.frame_count % 30 == 0:
            self._publish_status(f"Streaming OK - frame {self.frame_count}")

        if frame.dtype != np.uint8:
            frame = frame.astype(np.uint8)

        height, width = frame.shape[:2]

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = int(height)
        msg.width = int(width)

        if frame.ndim == 3 and frame.shape[2] == 3:
            msg.encoding = "bgr8"
            msg.is_bigendian = 0
            msg.step = int(width * 3)
            msg.data = frame.tobytes()
        elif frame.ndim == 2:
            msg.encoding = "mono8"
            msg.is_bigendian = 0
            msg.step = int(width)
            msg.data = frame.reshape(-1).tobytes()
        else:
            bgr = (
                cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
                if frame.shape[2] == 4
                else cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
            )
            msg.encoding = "bgr8"
            msg.is_bigendian = 0
            msg.step = int(width * 3)
            msg.data = bgr.reshape(-1).tobytes()

        self.publisher_.publish(msg)

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().debug(f"Status: {text}")

    def destroy_node(self) -> None:  # type: ignore[override]
        if hasattr(self, "cap") and self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RicohThetaPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
