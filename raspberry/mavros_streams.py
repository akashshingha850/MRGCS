#!/usr/bin/env python3
"""
mavros_streams.py — request ArduCopter telemetry streams through MAVROS.

ArduCopter only sends telemetry on a serial port when asked (or when the SRx_
parameters for that port are set). MAVROS does not request streams on its own,
so this node calls /mavros/set_stream_rate every time /mavros/state reports a
new FCU connection — including after a Pixhawk reboot.

Started alongside MAVROS by mavros_launch.sh.

Subscribes:  /mavros/state            mavros_msgs/State
Calls:       /mavros/set_stream_rate  mavros_msgs/srv/StreamRate
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy
from mavros_msgs.msg import State
from mavros_msgs.srv import StreamRate

# (stream_id, Hz) — same rates the old pymavlink bridge requested
STREAMS = [
    (StreamRate.Request.STREAM_ALL,          4),
    (StreamRate.Request.STREAM_EXTRA1,      10),  # ATTITUDE          → /mavros/imu/data
    (StreamRate.Request.STREAM_EXTRA2,       4),  # VFR_HUD           → /mavros/vfr_hud
    (StreamRate.Request.STREAM_POSITION,     5),  # GLOBAL_POSITION   → /mavros/global_position/*
    (StreamRate.Request.STREAM_RC_CHANNELS,  4),  # SERVO_OUTPUT_RAW  → /mavros/rc/out
]


class MavrosStreams(Node):

    def __init__(self):
        super().__init__('mavros_streams')
        self._connected = False
        self._requested = False
        self._cli = self.create_client(StreamRate, '/mavros/set_stream_rate')
        # /mavros/state is published transient-local
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(State, '/mavros/state', self._on_state, qos)
        # Retry until the service is discovered; re-arms after each disconnect
        self.create_timer(1.0, self._tick)

    def _on_state(self, msg: State):
        if not msg.connected:
            self._requested = False
        self._connected = msg.connected

    def _tick(self):
        if not self._connected or self._requested or not self._cli.service_is_ready():
            return
        self.get_logger().info('FCU connected — requesting telemetry streams')
        for sid, rate in STREAMS:
            req = StreamRate.Request(stream_id=sid, message_rate=rate, on_off=True)
            self._cli.call_async(req)
        self._requested = True


def main():
    rclpy.init()
    node = MavrosStreams()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):   # Ctrl+C / SIGTERM
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()   # SIGTERM may have shut the context down already


if __name__ == '__main__':
    main()
