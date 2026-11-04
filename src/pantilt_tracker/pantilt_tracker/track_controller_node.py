""" track_controller_node [2 pub] [service ser ???] [action server] [listener]

pub : /pan_tilt/joint_commands | Vector3 (intended to pan_tilt)
pub : /tracker/error | TrackingError (for whoever wants to listen it)

service ser : not doing this rn i have no clue what it should do

action ser : /tracker/center_target | CenterTarget

listener : pose that matter for us : 
            - target frame : whose eyes i am looking through : camera_optical_frame
            - source frame : what am i looking at : target_frame
"""
import time
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
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from pantilt_tracker_interfaces.msg import TrackingError
from pantilt_tracker_interfaces.srv import SetTrackingMode
from pantilt_tracker_interfaces.action import CenterTarget

class Tracker(Node):

    def __init__(self):

        super().__init__("track_controller_node")

        self.cb_group = ReentrantCallbackGroup()

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
            "/tracker/center_target",
            self._action_center_target_callback,
            callback_group=self.cb_group
        )

    def _action_center_target_callback(self,goal_handle):

        tolerance_deg = goal_handle.request.tolerance_deg

        try:
            t = self.tf_buffer.lookup_transform(
                target_frame="camera_optical_frame",
                source_frame="target_object",
                time=rclpy.time.Time()
            )
        except TransformException as e:
            self.get_logger().error(
                f"Could not transform 'camera_optical_frame' to 'target_frame'"
            )

        pan_error_deg = math.degrees(math.atan2(t.transform.translation.x,t.transform.translation.z))
        tilt_error_deg = math.degrees(math.atan2(-(t.transform.translation.y),t.transform.translation.z))

        self.get_logger().info(
            f"Goal received. Current error -> Pan: {pan_error_deg} deg, Tilt: {tilt_error_deg} deg"
        )

        feedback_msg = CenterTarget.Feedback()
        result = CenterTarget.Result()

        while True:

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger.warn(
                    "Goal was canceled by the client."
                )
                result.centered = False
                return result
            
            try:
                t = self.tf_buffer.lookup_transform(
                    target_frame="camera_optical_frame",
                    source_frame="target_frame",
                    time=rclpy.time.Time()
                )
            except TransformException as e:
                self.get_logger().error(
                    f"Could not transform 'camera_optical_frame' to 'target_frame'"
                )
                continue

            
            result.final_pan_deg = math.degrees(math.atan2(t.transform.translation.x,t.transform.translation.z))
            result.final_tilt_deg = math.degrees(math.atan2(t.transform.translation.y,t.transform.translation.z))
                
            msg_tracking_error = TrackingError()
            msg_tracking_error.pan_error_deg = math.degrees(
                math.atan2(
                    t.transform.translation.x,
                    t.transform.translation.z
                )
            )
            msg_tracking_error.tilt_error_deg = math.degrees(
                math.atan2(
                    -(t.transform.translation.y),
                    t.transform.translation.z
                )
            )
            msg_tracking_error.distance_to_target = self._calculate_distance_between_two_points(
                0,
                0,
                0,
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
            )
            self._pub_tracking_error.publish(msg_tracking_error)

            msg_joint_commands = Vector3()
            msg_joint_commands.x = 0.0
            msg_joint_commands.y = math.radians(msg_tracking_error.pan_error_deg)
            msg_joint_commands.z = math.radians(msg_tracking_error.tilt_error_deg)

            self._pub_joint_commands.publish(msg_joint_commands)

            line_of_sight = math.atan2(
                self._calculate_distance_between_two_points(
                    0,
                    0,
                    0,
                    t.transform.translation.x,
                    t.transform.translation.y,
                    0
                ),
                t.transform.translation.z,
            )
            if math.degrees(line_of_sight) < tolerance_deg:
                self.get_logger().info(
                    f"Target aligned within {tolerance_deg}! Goal Succeeded."
                )
                goal_handle.succeed()
                result.centered = True
                return result

            feedback_msg.current_pan_error_deg = math.degrees(
                math.atan2(
                    t.transform.translation.x,
                    t.transform.translation.z
                )
            )
            feedback_msg.current_tilt_error_deg = math.degrees(
                math.atan2(
                    -(t.transform.translation.y),
                    t.transform.translation.z
                )
            )
            self.get_logger().info(
                f"[Feedback] Pan Error : {feedback_msg.current_pan_error_deg} | Tilt Error : {feedback_msg.current_tilt_error_deg}"
            )
            goal_handle.publish_feedback(feedback_msg)

            time.sleep(0.5)

    def _calculate_distance_between_two_points(self,x1=0.0,y1=0.0,z1=0.0,x2=0.0,y2=0.0,z2=0.0) -> float:
        d_x = x2-x1
        d_y = y2-y1
        d_z = z2-z1
        return math.sqrt(d_x**2  + d_y**2 + d_z**2)

    def _calculate_angle_between_two_points(self,x1,y1,x2,y2) -> float:
        d_x = x2-x1
        d_y = y2-y1
        return math.atan2(d_y,d_x)

def main():
    rclpy.init()
    node = Tracker()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()