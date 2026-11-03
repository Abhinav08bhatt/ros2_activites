""" track_controller_node [2 pub] [service ser ???] [action server] [listener]

pub : /pan_tilt/joint_commands | Vector3 (intended to pan_tilt)
pub : /tracker/error | TrackingError (for whoever wants to listen it)

service ser : not doing this rn i have no clue what it should do

action ser : /tracker/center_target | CenterTarget

listener : pose that matter for us : 
            - target frame : whose eyes i am looking through : camera_optical_frame
            - source frame : what am i looking at : target_frame
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3

from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException

from rclpy.action import ActionServer

from pantilt_tracker_interfaces.msg import TrackingError
from pantilt_tracker_interfaces.srv import SetTrackingMode
from pantilt_tracker_interfaces.action import CenterTarget

class Tracker(Node):

    def __init__(self):

        super().__init__("track_controller_node")

        self.tf_buffer = Buffer()
        self._tf_listener = TransformListener(self.tf_buffer,self)

        self._pub_joint_commands = self.create_publisher(
            Vector3,
            "/pan_tilt/joint_commands",
            10
        )
        
        self._pub_tracking_error = self.create_publisher(
            TrackingError,
            "/tracker/error",
            10
        )

        self._action_center_target = ActionServer(
            self,
            CenterTarget,
            "/target/center_target",
            self._action_center_target_callback
        )

    def _action_center_target_callback(self,goal_handle):

        # needs to handle all of it....response feedback result calculations and idk what not
        
        pass