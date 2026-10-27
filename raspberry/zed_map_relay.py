#!/usr/bin/env python3
"""
zed_map_relay.py — ZED spatial-mapping cloud, cut down for the GCS.

The ZED wrapper's mapping/fused_cloud is the whole map in one message (millions
of points, ~70 MB at 5 cm voxels, 1 Hz) — far too much for rosbridge. This node
voxel-downsamples it (xyz float32, plus the ZED's packed `rgb` colour when the cloud has it) and
republishes it at most every `period` s.

It subscribes to the fused cloud only while something (rosbridge for the GCS)
subscribes to gcs/map_cloud, so an idle GCS costs the ZED stack nothing.

Topic names are relative: run it in the drone's namespace
(`--ros-args -r __ns:=/drone_1` next to bisg_isaac's ZED stack).

Subscribes:  zed/zed_node/mapping/fused_cloud  sensor_msgs/PointCloud2  (frame map)
Publishes:   gcs/map_cloud                     sensor_msgs/PointCloud2  xyz float32 (+ rgb, 4 bytes B,G,R,A like the input), same frame

Parameters:
  voxel       0.2      m — downsampling grid (the bisg warehouse: ~4.2 M → ~120 k points)
  max_points  200000   hard cap (random subset above it)
  period      2.0      s — minimum time between published maps
"""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField

IN_TOPIC  = 'zed/zed_node/mapping/fused_cloud'
OUT_TOPIC = 'gcs/map_cloud'

# Depth 1: a 70 MB map that is already stale must not queue behind the next one
IN_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                    history=HistoryPolicy.KEEP_LAST, depth=1)

XYZ_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
RGB_FIELD = PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)   # packed colour, bytes kept as received


class ZedMapRelay(Node):

    def __init__(self):
        super().__init__('zed_map_relay')
        self._voxel  = self.declare_parameter('voxel', 0.2).value
        self._max    = self.declare_parameter('max_points', 200000).value
        self._period = self.declare_parameter('period', 2.0).value

        self._pub  = self.create_publisher(PointCloud2, OUT_TOPIC, 1)
        self._sub  = None
        self._last = 0.0
        self.create_timer(1.0, self._follow_demand)
        self.get_logger().info(
            f'zed_map_relay ready — voxel {self._voxel} m, max {self._max} pts, every {self._period} s')

    def _follow_demand(self):
        """Subscribe upstream only while the relayed map has a subscriber."""
        wanted = self._pub.get_subscription_count() > 0
        if wanted and self._sub is None:
            self._sub = self.create_subscription(PointCloud2, IN_TOPIC, self._on_cloud, IN_QOS)
            self.get_logger().info('GCS subscribed — relaying the ZED map')
        elif not wanted and self._sub is not None:
            self.destroy_subscription(self._sub)
            self._sub = None
            self.get_logger().info('no subscribers — ZED map relay idle')

    def _on_cloud(self, msg: PointCloud2):
        now = time.monotonic()
        if now - self._last < self._period:
            return
        self._last = now

        n = msg.width * msg.height
        if n == 0:
            return
        off = {f.name: f.offset for f in msg.fields}
        if not {'x', 'y', 'z'} <= off.keys():
            self.get_logger().warn('fused_cloud has no x/y/z fields', once=True)
            return
        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)
        xyz = np.empty((n, 3), dtype=np.float32)
        for i, k in enumerate('xyz'):
            xyz[:, i] = rows[:, off[k]:off[k] + 4].copy().view(np.float32).ravel()
        # the colour travels as raw bytes (a float32 field holding packed B,G,R,A; never read as a number)
        rgb = rows[:, off['rgb']:off['rgb'] + 4].copy().view('<u4').ravel() if 'rgb' in off else None
        finite = np.isfinite(xyz).all(axis=1)
        xyz = xyz[finite]
        if rgb is not None:
            rgb = rgb[finite]
        if len(xyz) == 0:
            return

        # One point per occupied voxel (voxel index packed into one int64: a 1-D unique is far faster)
        keys = np.floor(xyz / self._voxel).astype(np.int64)
        keys -= keys.min(axis=0)
        dims = keys.max(axis=0) + 1
        lin = (keys[:, 0] * dims[1] + keys[:, 1]) * dims[2] + keys[:, 2]
        _, first = np.unique(lin, return_index=True)
        if len(first) > self._max:
            first = np.random.default_rng().choice(first, self._max, replace=False)
        voxels = len(first)
        pts = np.empty(len(first), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4')] + ([('rgb', '<u4')] if rgb is not None else []))
        pts['x'], pts['y'], pts['z'] = xyz[first, 0], xyz[first, 1], xyz[first, 2]
        if rgb is not None:
            pts['rgb'] = rgb[first]

        out = PointCloud2()
        out.header       = msg.header
        out.height       = 1
        out.width        = len(pts)
        out.fields       = XYZ_FIELDS + ([RGB_FIELD] if rgb is not None else [])
        out.is_bigendian = False
        out.point_step   = pts.dtype.itemsize
        out.row_step     = out.point_step * len(pts)
        out.is_dense     = True
        out.data         = pts.tobytes()
        self._pub.publish(out)
        self.get_logger().info(
            f'map {n} points → {voxels} voxels → {len(pts)} sent in {time.monotonic() - now:.2f} s',
            throttle_duration_sec=30.0)


def main():
    rclpy.init()
    node = ZedMapRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):   # Ctrl+C / SIGTERM
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()   # SIGTERM may have shut the context down already


if __name__ == '__main__':
    main()
