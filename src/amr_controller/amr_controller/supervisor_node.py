'''arm_node [pub] [sub] [service server] [action ser]

# DEFINED VALUE(s)
    - variable = value ()

PARAMETER:
    - mode = NORMAL (or BRAKE)

PUBLISHER:
    - TYPE: Pose2DStamped | TOPIC: /amr/odometry
    - MSG:
        - x : current_x
        - y : current_y
        - theta : current_theta
        - linear_velocity : current_linear_velocity
        - angular_velocity : current_angular_velocity
    - Logs current dash-board information
        
SUBSCRIBER: (i dont get why we need to use this tbh so i am leaving this)
    - TYPE: SpeedLimit | TOPIC: /amr/speed_limit
    - MSG:
        - max_linear_speed : 0.2 
    - gives max_linear_speed when we are 1 m near the goal
        
SERVICE SERVER:
    - TYPE: EmergencyBreak | TOPIC: /amr/emergency_break
    - SRV:
        - REQUEST:
            - engage_break : bool
        - RESPONSE:
            - success : bool
            - message : string
    - triggers after 5 sec if param mode is BREAK

ACTION SERVER:
    - TYPE: NavigateToPose | TOPIC: /amr/navigate_to_pose
    - ACTION:
        - GOAL:
            - goal_x : 
            - goal_y : 
            - tolerance : 
        - RESULT: 
            - reached : 
            - final_x : 
            - final_y : 
            - final_theta : 
            - termination_reason : 
        - FEEDBACK:
            - current_x : 
            - current_y : 
            - current_theta : 
            - distance remaining : 
'''

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from action_msgs.msg import GoalStatus
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

from amr_interface.msg import Pose2DStamped, SpeedLimit
from amr_interface.srv import EmergencyBreak
from amr_interface.action import NavigateToPose


class Supervisor(Node):

    def __init__(self):

    #* node name : supervisor_node
        super().__init__("supervisor_node")

    #* parameter
        self.declare_parameter(
            "mode",
            "NORMAL",
            ParameterDescriptor(description = "Default : NORMAL\nOptions : NORMAL , BREAK")
        )

        self.declare_parameter(
            "goal_x",
            2.0,
            ParameterDescriptor(description = "Default : 10.0 (Float)")
        )
        self.declare_parameter(
            "goal_y",
            2.0,
            ParameterDescriptor(description = "Default : 10.0 (Float)")
        )
        self.declare_parameter(
            "tolerance",
            0.1,
            ParameterDescriptor(description = "Default : 0.5 (Float)")
        )
        self.mode = self.get_parameter("mode").value
        self.goal_x = self.get_parameter("goal_x").value
        self.goal_y = self.get_parameter("goal_y").value
        self.tolerance = self.get_parameter("tolerance").value
        
        self.add_on_set_parameters_callback(self.parameter_change_callback)

    #* flag variable to know if we have sent the action or not
        self._act_navigation_goal_sent = False

    #* node starting message
        self.get_logger().info(
            f"DISPATCHING GOAL | Pose : ({self.goal_x},{self.goal_x}) | Tolerance: {self.tolerance}m"
        )

    #* subscriber
        self._sub_odometer = self.create_subscription(
            Pose2DStamped,
            "/amr/odometry",
            self._sub_odometer_callback_logger,
            10
        )

    #* service client (kicks in 5 sec after node startup if the mode is BREAK)
        self._ser_emergency_break_cli = self.create_client(
            EmergencyBreak,
            "/amr/emergency_break"
        )
        if self.mode == "BREAK":
            self.emergency_break_cli_timer = self.create_timer(
                5.0,
                self.emergency_break_cli_callback
            )

    #* action client
        self._act_navigate_goal_cli = ActionClient(
            self,
            NavigateToPose,
            "/amr/navigate_to_pose"
        )

# ==================================== sub callback (log odometer) ===================================
    def _sub_odometer_callback_logger(self,msg):
        self.get_logger().info(
            f"[SUB] Pose : ({msg.x:.2f} , {msg.y:.2f} , {msg.theta:.2f}) | v : {msg.linear_velocity:.2f} w : {msg.angular_velocity:.2f}"
        )

        if not self._act_navigation_goal_sent:
            self._act_navigation_goal_sent = True
            self._act_navigate_goal_callback()

# ========================== action (callback + response + feedback + result) ========================
    def _act_navigate_goal_callback(self):

        goal_msg = NavigateToPose.Goal()
        goal_msg.goal_x = self.goal_x
        goal_msg.goal_y = self.goal_y
        goal_msg.tolerance = self.tolerance

        self._act_navigate_goal_cli.wait_for_server()

        self.goal_future = self._act_navigate_goal_cli.send_goal_async(
            goal_msg,
            feedback_callback = self._act_navigate_goal_feedback
        )
        self.goal_future.add_done_callback(self._act_navigate_goal_response)

    def _act_navigate_goal_response(self,future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(
                "[ACT ERROR] Goal rejected | need manual fix"
            )
            return
        
        self.get_logger().info(
            "[ACT CLI] NAV GOAL ACCEPTED | Starting action..."
        )

        self.future_result = goal_handle.get_result_async()
        self.future_result.add_done_callback(self._act_navigate_goal_result)

    def _act_navigate_goal_feedback(self,feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"[ACT CLI FEED] Navigating | Pose : ({feedback.current_x:.2f},{feedback.current_y:.2f},{feedback.current_theta:.2f}) | Remaining Distance : {feedback.distance_remaining:.2f} "
        )        

    def _act_navigate_goal_result(self,future):

        status = future.result().status
        result = future.result().result

        if status == GoalStatus.STATUS_SUCCEEDED and result.reached:
            self.get_logger().info(
                f"[ACT RESULT] SUCCESS : {result.termination_reason} | Pose : ({result.final_x:.2f},{result.final_y:.2f},{result.final_theta:.2f}) "
            )
        elif status == GoalStatus.STATUS_ABORTED or not result.reached:
            self.get_logger().error(
                f"[ACT RESULT] MISSION ABORTED | Reason : {result.termination_reason} | Pose : ({result.final_x:.2f},{result.final_y:.2f},{result.final_theta:.2f})"
            )
        else:
            self.get_logger().fatal(
                "[ACT RESULT] Something is wrong | job not completed | need manual fix"
            )

# ===================================== service cli (5.0 | BREAK) ====================================
    def emergency_break_cli_callback(self):
        # get triggered after 5 sec if the mde is BREAK
        pass

# ======================================= helper function (AI) =======================================
    def parameter_change_callback(self,params):
        """AI WRITTEN FUNCTION : changes the actual value of parameter when changed from terminal

        Args:
            params (class object): ros2 handles this no need to worry

        Returns:
            custom_interface: its pre built in ros we dont need to worry
        """
        for param in params:
            if hasattr(self, param.name):
                setattr(self, param.name, param.value)
                self.get_logger().info(
                    f"PARAMETER [ {param.name} ] updated to : [ {param.value} ]"
                )
        return SetParametersResult(successful=True)



def main(args=None):
    rclpy.init(args=args)
    node = Supervisor()    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()