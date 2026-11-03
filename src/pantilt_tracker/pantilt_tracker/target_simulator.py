import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster

class Target(Node):

    def __init__(self):

        super().__init__("target_simulator")

        self._static_broadcaster = StaticTransformBroadcaster(self)
        self._dynamic_broadcaster = TransformBroadcaster(self)

        self.publish_static_conveyer_belt()

        self._timer = self.create_timer(
            0.05,
            self.publish_dynamic_target_frame
        )
        self.start_time = self.get_clock().now()

    def publish_static_conveyer_belt(self):

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "conveyer_belt"

        t.transform.translation.x = 2.0
        t.transform.translation.y = -0.5
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self._static_broadcaster.sendTransform(t)

    def publish_dynamic_target_frame(self):

        now_time = self.get_clock().now()
        time_elapsed_raw = now_time - self.start_time
        time_elapsed = time_elapsed_raw.nanoseconds / 1e9

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "conveyer_belt"
        t.child_frame_id = "target_frame"

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.8 * math.sin(0.5 * time_elapsed)
        t.transform.translation.z = 0.2

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self._dynamic_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = Target()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()