#!/usr/bin/env python3
"""
zed_image_relay.py — ZED depth and confidence maps, and the sim's chase camera, as small JPEGs for the GCS.

The ZED wrapper's depth_registered and confidence_map are raw 32-bit float images (3.7 MB each at 720p): far too much for
rosbridge and a phone. This node colour-maps them (turbo), shrinks them and republishes them as JPEG CompressedImage, a few
per second. The RGB image needs no relay: the GCS reads the wrapper's own left/.../image/compressed.

Like zed_map_relay.py it subscribes upstream only while its output has a subscriber, so an idle GCS costs the ZED stack nothing.

Topic names are relative: run it in the drone's namespace (`--ros-args -r __ns:=/drone_1`).

Subscribes:  chase/image                                 sensor_msgs/Image  rgb8, the sim's third-person camera (absent on the real drone)
             zed/zed_node/depth/depth_registered         sensor_msgs/Image  32FC1, metres (NaN = no return)
             zed/zed_node/confidence/confidence_map      sensor_msgs/Image  32FC1, 0..100, higher = LESS reliable
Publishes:   gcs/chase/compressed                        sensor_msgs/CompressedImage  the chase view as is (Kinetix tab live view)
             gcs/depth/compressed                        sensor_msgs/CompressedImage  turbo: blue = near, red = far
             gcs/confidence/compressed                   sensor_msgs/CompressedImage  turbo: blue = reliable, red = unreliable

Parameters:
  width       480    px — output width (height keeps the aspect ratio)
  rate_hz     4.0    maximum images per second per stream
  chase_rate_hz 8.0  the chase view is a live picture, so it gets more
  min_depth   0.3    m — depth shown as blue
  max_depth   10.0   m — depth shown as red
  quality     80     JPEG quality
"""

import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

STREAMS = {   # name -> (input topic, output topic)
    'depth':      ('zed/zed_node/depth/depth_registered', 'gcs/depth/compressed'),
    'confidence': ('zed/zed_node/confidence/confidence_map', 'gcs/confidence/compressed'),
    'chase':      ('chase/image', 'gcs/chase/compressed'),
}
IN_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)


def colorize(a: np.ndarray, lo: float, hi: float, invalid=(0, 0, 0)) -> np.ndarray:
    """float image -> BGR turbo, lo = blue, hi = red; non-finite pixels are painted `invalid`."""
    ok = np.isfinite(a)
    t = np.clip((np.where(ok, a, lo) - lo) / (hi - lo), 0.0, 1.0)
    bgr = cv2.applyColorMap((t * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    bgr[~ok] = invalid
    return bgr


class ZedImageRelay(Node):

    def __init__(self):
        super().__init__('zed_image_relay')
        self._width   = int(self.declare_parameter('width', 480).value)
        self._period  = 1.0 / float(self.declare_parameter('rate_hz', 4.0).value)
        self._chase_period = 1.0 / float(self.declare_parameter('chase_rate_hz', 8.0).value)
        self._min_d   = float(self.declare_parameter('min_depth', 0.3).value)
        self._max_d   = float(self.declare_parameter('max_depth', 10.0).value)
        self._quality = int(self.declare_parameter('quality', 80).value)

        self._pub, self._sub, self._last = {}, {}, {}
        for name, (_, out) in STREAMS.items():
            self._pub[name]  = self.create_publisher(CompressedImage, out, 1)
            self._sub[name]  = None
            self._last[name] = 0.0
        self.create_timer(1.0, self._follow_demand)
        self.get_logger().info(f'zed_image_relay ready — {self._width} px wide, {1 / self._period:g} Hz max')

    def _follow_demand(self):
        for name, (inp, _) in STREAMS.items():
            wanted = self._pub[name].get_subscription_count() > 0
            if wanted and self._sub[name] is None:
                self._sub[name] = self.create_subscription(Image, inp, lambda m, n=name: self._on_image(n, m), IN_QOS)
                self.get_logger().info(f'GCS subscribed — relaying {name}')
            elif not wanted and self._sub[name] is not None:
                self.destroy_subscription(self._sub[name])
                self._sub[name] = None
                self.get_logger().info(f'no subscribers — {name} relay idle')

    def _on_image(self, name: str, msg: Image):
        now = time.monotonic()
        if now - self._last[name] < (self._chase_period if name == 'chase' else self._period):
            return
        self._last[name] = now
        if name == 'chase':
            if msg.encoding != 'rgb8' or not msg.width or not msg.height:
                self.get_logger().warn(f'chase: expected rgb8, got {msg.encoding!r}', once=True)
                return
            img = cv2.cvtColor(np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3), cv2.COLOR_RGB2BGR)
            h = max(1, round(msg.height * self._width / msg.width))
            ok, buf = cv2.imencode('.jpg', cv2.resize(img, (self._width, h)), [cv2.IMWRITE_JPEG_QUALITY, self._quality])
            if ok:
                out = CompressedImage()
                out.header, out.format, out.data = msg.header, 'jpeg', buf.tobytes()
                self._pub[name].publish(out)
            return
        if msg.encoding != '32FC1' or not msg.width or not msg.height:
            self.get_logger().warn(f'{name}: expected 32FC1, got {msg.encoding!r}', once=True)
            return
        a = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        if name == 'depth':
            img = colorize(a, self._min_d, self._max_d)
        else:
            img = colorize(a, 0.0, 100.0)
        h = max(1, round(msg.height * self._width / msg.width))
        img = cv2.resize(img, (self._width, h), interpolation=cv2.INTER_NEAREST)     # nearest: keep no-return pixels crisp
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, self._quality])
        if not ok:
            return
        out = CompressedImage()
        out.header, out.format, out.data = msg.header, 'jpeg', buf.tobytes()
        self._pub[name].publish(out)


def main():
    rclpy.init()
    node = ZedImageRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):   # Ctrl+C / SIGTERM
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
