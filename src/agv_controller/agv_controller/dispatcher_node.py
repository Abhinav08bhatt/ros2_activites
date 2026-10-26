'''dispatcher_node.py | [sub] [service cli] [action cli] |

PARAMETERS: 
    - goal_x        : (6.0)
    - goal_y        : (6.0)
    - cargo_weight  : (10.0) (kg)

SUBSCRIBER:
    - TYPE : AgvTelemetry | TOPIC : /agv/telemetry
    - LOGS : 
        - current_position
        - battery (warn if battery under 20%)
    
SERVICE CLIENT: 
    - TYPE : ValidateWaypoint | TOPIC : /agv/validate_waypoint
    - kicks after 3.0 sec of startup of the Node
    - gives the target_x and target_y through the topic
    - receives the response from the server
    -   is_valid: False | reason: string
    -       logs as error[reason] and about mission
    -   is_valid: True | estimated_distance: float
    -       logs as success and kicks in the action client
    
ACTION SERVER:
    - TYPE : NavigateToWaypoint | TOPIC : /agv/navigate_to_waypoint
    - GOAL:
    -   target_x = x coordinate of target
    -   target_y = y coordinate of target
    -   payload_weight = weight 
    - FEEDBACK: 
    -   distance_tp_goal = target_distance - (speed*time)
    -   current_battery = float_value
    - RESULT:
    -   reached = True/False
    -   total_distance_moved = target_distance if true else figure it out
    -   remaining_battery = float
'''


import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus

from rcl_interfaces.msg import ParameterDescriptor,SetParametersResult

from agv_interfaces.msg import AgvTelemetry
from agv_interfaces.srv import ValidateWaypoint
from agv_interfaces.action import NavigateToWaypoint

class Dispatcher(Node):

    def __init__(self):

    # node name : dispatcher_node
        super().__init__("dispatcher_node")

    # hardcoded variable(s)
        # self.variable = value 

    # parameters
        self.declare_parameter(
            "goal_x",
            6.0,
            ParameterDescriptor(description = "x coordinate of goal (float) [default : 6.0]")
        )
        self.declare_parameter(
            "goal_y",
            6.0,
            ParameterDescriptor(description = "y coordinate of goal (float) [default : 6.0]")
        )
        self.declare_parameter(
            "cargo_weight",
            10.0,
            ParameterDescriptor(description = "weight of rover in kg (float) [default : 10.0]")
        )
        self.goal_x = self.get_parameter("goal_x").value
        self.goal_y = self.get_parameter("goal_y").value
        self.cargo_weight = self.get_parameter("cargo_weight").value

    # parameter caller
        self.add_on_set_parameters_callback(self.parameter_change_callback)

    # logging start of NODE
        self.get_logger().info(
            f"DISPATCHER INITIALIZED | Target : ({self.goal_x} , {self.goal_y}) | Cargo: {self.cargo_weight} kg |"
        )

    # subscriber
        self._sub = self.create_subscription(
            AgvTelemetry,
            "/agv/telemetry",
            self.agv_pub_contact,
            10
        )

    # service client
        self._service_validate_goal_client = self.create_client(
            ValidateWaypoint,
            "/agv/validate_waypoint"
        )
    # timer to kick the service client to go check if target is safe to processed or not 
        self.dispatch_service_validation_timer = self.create_timer(
            3.0,
            self.dispatch_service_validation
        )

    # action client
        self._action_navigate_goal_client = ActionClient(
            self,
            NavigateToWaypoint,
            '/agv/navigate_to_waypoint'
        )

        
    def agv_pub_contact(self,msg):
        if msg.current_battery <= 20:
            self.get_logger().warn(
                f"[TELEMETRY] | Pos: ({msg.current_x:.1f},{msg.current_y:.1f}) | Battery: {msg.current_battery:.1f} [CRITICAL] | Status: {msg.status} |"
            )
        else:
            self.get_logger().info(
                f"[TELEMETRY] | Pos: ({msg.current_x:.1f},{msg.current_y:.1f}) | Battery: {msg.current_battery:.1f} | Status: {msg.status} |"
            )

    def dispatch_service_validation(self):
        # get triggered after 3.0 sec of node startup and runs once only
        self.dispatch_service_validation_timer.cancel()
        
        self.get_logger().info(
            f"VALIDATING WAYPOINT with AGV service | Target coordinates : ({self.goal_x:.1f},{self.goal_y:.1f}) |"
        )
        request = ValidateWaypoint.Request()
        request.target_x = self.goal_x
        request.target_y = self.goal_y

        future = self._service_validate_goal_client.call_async(request)
        future.add_done_callback(self.dispatch_service_validation_callback)

    def dispatch_service_validation_callback(self,future):

        try:
            response = future.result()
            if response.is_valid:
                self.get_logger().info(
                    f"SERVICE RESPONSE | Valid: {response.is_valid} | Est. Distance: {response.estimated_distance:.1f}"
                )
                self.dispatch_action_goal_callback()
            elif response.reason == "ALREADY ON SITE":
                self.get_logger().fatal(
                    "MISSION TRASHED | Proposed destination is already reached."
                )
                return
            elif response.reason == "BATTERY DIED":
                self.get_logger().fatal(
                    "BATTERY DIED | CANNOT GO ANY FURTHER | Validation DISAPPROVED |"

                )
                return
            else :
                self.get_logger().error(
                    f"SERVICE RESPONSE | Valid: {response.is_valid} | Reason: {response.reason} "
                )
                self.get_logger().fatal(
                    f"MISSION ABORTED | Proposed destination is unsafe. Action will not be dispatched |"
                )
                return
        except Exception as e:
            self.get_logger().fatal(f"SERVICE FAILED | {e}")

    def dispatch_action_goal_callback(self):

        goal_msg = NavigateToWaypoint.Goal()
        goal_msg.target_x = self.goal_x
        goal_msg.target_y = self.goal_y
        goal_msg.payload_weight = self.cargo_weight

        self._action_navigate_goal_client.wait_for_server()

        self.self_goal_future = self._action_navigate_goal_client.send_goal_async(
            goal_msg,
            feedback_callback = self.dispatch_action_feedback_callback
        )

        self.get_logger().info(
            "WAYPOINT APPROVED | Dispatching action goal..."
        )

        self.self_goal_future.add_done_callback(self.dispatch_action_response_callback)

    def dispatch_action_response_callback(self,future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().fatal("GOAL REJECTED | need manual fix")
            return

        self._future_result = goal_handle.get_result_async()
        self._future_result.add_done_callback(self.dispatch_action_result_callback)

    def dispatch_action_feedback_callback(self,feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"[FEEDBACK] Distance to goal: {feedback.distance_to_goal:.1f} | Battery: {feedback.current_battery:.1f} |"
        )

    def dispatch_action_result_callback(self,future):

        status = future.result().status
        result = future.result().result
        
        if status == GoalStatus.STATUS_SUCCEEDED and result.reached:
            self.get_logger().info(
                "[RESULT] MISSION COMPLETED"
            )
            self.get_logger().info(
                f"[RESULT] Total distance traveled: {result.total_distance_traveled:.1f} | Battery: {result.remaining_battery:.1f} | Success: {result.reached}"
            )
        elif status == GoalStatus.STATUS_ABORTED or not result.reached:
            self.get_logger().error(
                f"[RESULT] MISSION ABORTED | AGV DIED | Battery : {result.remaining_battery:.1f}% | Distance remaining : {result.distance_remaining:.1f}m "
            )
        else:
            self.get_logger().fatal(
                "[RESULT] Something is shit wrong | need manual assistance.."
            )

    # helper function (AI WRITTEN)
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
    node = Dispatcher()    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()