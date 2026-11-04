""" pan_tile_broadcaster [sub] [broadcaster {2 static + 2 dynamic}]

sub : /pan_tilt/joint_commands | geometry_msgs/msg/Vector3
    - geometry_msgs/msg/Vector3 :
        float64 x
        float64 y
        float64 z

broadcaster : 
    - (static) : riser_link         (x = 0 , y = 0 , z = 1 | x = 0,     y = 0,                  z = 0,                  w = 1)
    - (dynamic) : pan_link          (x = 0 , y = 0 , z = 0 | x = 0,     y = 0,                  z = sin(pan_angle/2.0), w = sin(pan_angle/2.0))
    - (dynamic) : tilt_link         (x = 0 , y = 0 , z = 0 | x = 0,     y = sin(tilt_angle/2.0), z = 0,                  w = sin(tilt_angle/2.0))
    - (static) : camera_optic_frame (x = 0 , y = 0 , z = 0 | x = -0.5,  y = 0.5,                z = -0.5,               w = 0.5)    
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster

class PanTilt(Node):

    def __init__(self):

        super().__init__("pan_tilt_broadcaster")

        self._static_broadcaster = StaticTransformBroadcaster(self)
        self._dynamic_broadcaster = TransformBroadcaster(self)


        self._sub_joint_commands = self.create_subscription(
            Vector3,
            "/pan_tilt/joint_commands",
            self._update_transform,
            10
        )

        self._publish_static_transforms()
        self._publish_dynamic_transforms_timer = self.create_timer(
            0.05,
            self._publish_dynamic_transforms
        )

        self.pan_angle = 0.0
        self.tilt_angle = 0.0

    def _update_transform(self,msg):

        self.pan_angle = msg.y
        self.tilt_angle = msg.z

    def _publish_static_transforms(self):

        transforms = []

        now = self.get_clock().now().to_msg()

        t_riser = TransformStamped()
        t_riser.header.stamp = now
        t_riser.header.frame_id = "world"
        
        t_riser.child_frame_id = "riser_link"

        t_riser.transform.translation.x = 0.0
        t_riser.transform.translation.y = 0.0
        t_riser.transform.translation.z = 1.0

        t_riser.transform.rotation.x = 0.0
        t_riser.transform.rotation.y = 0.0
        t_riser.transform.rotation.z = 0.0
        t_riser.transform.rotation.w = 1.0

        transforms.append(t_riser)
        
        t_cam = TransformStamped()
        t_cam.header.stamp = now
        t_cam.header.frame_id = "tilt_link"
        
        t_cam.child_frame_id = "camera_optical_frame"

        t_cam.transform.translation.x = 0.0
        t_cam.transform.translation.y = 0.0
        t_cam.transform.translation.z = 0.0

        t_cam.transform.rotation.x = -0.5
        t_cam.transform.rotation.y = 0.5
        t_cam.transform.rotation.z = -0.5
        t_cam.transform.rotation.w = 0.5

        transforms.append(t_cam)

        self._static_broadcaster.sendTransform(transforms)


    def _publish_dynamic_transforms(self):

        transforms = []

        now = self.get_clock().now().to_msg()

        t_pan = TransformStamped()
        t_pan.header.stamp = now
        t_pan.header.frame_id = "riser_link"
        
        t_pan.child_frame_id = "pan_link"

        t_pan.transform.translation.x = 0.0
        t_pan.transform.translation.y = 0.0
        t_pan.transform.translation.z = 0.0

        t_pan.transform.rotation.x = 0.0
        t_pan.transform.rotation.y = 0.0
        t_pan.transform.rotation.z = math.sin(self.pan_angle/2)
        t_pan.transform.rotation.w = math.cos(self.pan_angle/2)

        transforms.append(t_pan)
        
        t_tilt = TransformStamped()
        t_tilt.header.stamp = now
        t_tilt.header.frame_id = "pan_link"
        
        t_tilt.child_frame_id = "tilt_link"

        t_tilt.transform.translation.x = 0.0
        t_tilt.transform.translation.y = 0.0
        t_tilt.transform.translation.z = 0.0

        t_tilt.transform.rotation.x = 0.0
        t_tilt.transform.rotation.y = math.sin(self.tilt_angle/2)
        t_tilt.transform.rotation.z = 0.0
        t_tilt.transform.rotation.w = math.cos(self.tilt_angle/2)

        transforms.append(t_tilt)

        self._dynamic_broadcaster.sendTransform(transforms)

def main():
    rclpy.init()
    node = PanTilt()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()