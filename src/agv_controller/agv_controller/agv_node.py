'''agv_node.py | [pub] [service ser] [action ser] |

PARAMETERS: 
    - spawn_x           : (0.0)
    - spawn_y           : (0.0)
    - linear_speed      : (1.0) (m/s)
    - hazard_center_x   : (5.0)
    - hazard_center_y   : (5.0)
    - hazard_radius     : (2.0)
    - base_energy_rate  : (0.5) (%/m)

PUBLISHER:
    - timer : 0.5
    - TYPE : AgvTelemetry | TOPIC : /agv/telemetry
    - MSG:
    -   current_x : current x coordinate of position of rover
    -   current_y : current y coordinate of position of rover
    -   current_battery : current battery of rover
    -   Status : IDLE, NAVIGATING, HAZARD_STOP

SERVICE SERVER:  
    - TYPE : ValidateWaypoint | TOPIC : /agv/validate_waypoint
    - calculate distance from current position to target position (pythagoras theorem)
    - checks if target falls inside the circular zone: 
    -   IF INSIDE:
    -       is_valid = False
    -       reason = Inside Hazard Zone 
    -   IF NOT INSIDE:
    -       is_valid = True
    -       estimated_distance = distance from current position to target position

ACTION SERVER:
    - TYPE : NavigateToWaypoint | TOPIC : /agv/navigate_to_waypoint
    - Moves the robot towards the target in a loop at 1 sec increments and updates battery 
    - GOAL:
    -   target_x = x coordinate of target
    -   target_y = y coordinate of target
    -   payload_weight = weight 
    - FEEDBACK: 
    -   distance_to_goal = target_distance - (speed*time)
    -   current_battery = float_value
    - RESULT:
    -   reached = True/False
    -   total_distance_moved = target_distance if true else figure it out
    -   remaining_battery = float
'''


import time, math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from rcl_interfaces.msg import ParameterDescriptor,SetParametersResult

from agv_interfaces.msg import AgvTelemetry
from agv_interfaces.srv import ValidateWaypoint
from agv_interfaces.action import NavigateToWaypoint

class Agv(Node):

    def __init__(self):

    # node name : agv_node
        super().__init__("agv_node")

    # creating a callback group
        self.cb_group = ReentrantCallbackGroup()

    # parameters
        self.declare_parameter(
            "spawn_x",
            0.0,
            ParameterDescriptor(description = "spawn x coordinate of rover (float) [default : 0.0]")
        )
        self.declare_parameter(
            "spawn_y",
            0.0,
            ParameterDescriptor(description = "spawn y coordinate of rover (float) [default : 0.0]")
        )
        self.declare_parameter(
            "linear_speed",
            1.0,
            ParameterDescriptor(description = "speed of rover in meter/sec (float) [default : 1.0]")
        )
        self.declare_parameter(
            "hazard_center_x",
            5.0,
            ParameterDescriptor(description = "x coordinate of middle of hazard circle (float) [default : 5.0]")
        )
        self.declare_parameter(
            "hazard_center_y",
            5.0,
            ParameterDescriptor(description = "y coordinate of middle of hazard circle (float) [default : 5.0]")
        )
        self.declare_parameter(
            "hazard_radius",
            2.0,
            ParameterDescriptor(description = "radius of hazard circle (float) [default : 2.0]")
        )
        self.declare_parameter(
            "base_energy_rate",
            0.5,
            ParameterDescriptor(description = "rate at which battery dec in % per meter (float) [default : 0.5]")
        )
        self.spawn_x = self.get_parameter("spawn_x").value
        self.spawn_y = self.get_parameter("spawn_y").value
        self.linear_speed = self.get_parameter("linear_speed").value
        self.hazard_center_x = self.get_parameter("hazard_center_x").value
        self.hazard_center_y = self.get_parameter("hazard_center_y").value
        self.hazard_radius = self.get_parameter("hazard_radius").value
        self.base_energy_rate = self.get_parameter("base_energy_rate").value

    # parameter caller
        self.add_on_set_parameters_callback(self.parameter_change_callback)

    # hardcoded variable(s)
        self.battery = 100.0
        self.current_x = self.spawn_x
        self.current_y = self.spawn_y

    # logging start of NODE
        self.get_logger().info(
            f"AGV ONLINE | Initial Pose: ({self.spawn_x} , {self.spawn_y}) | Battery: {self.battery}% |"
        )
        self.get_logger().warn(
            f"HAZARD ZONE LOCATION FOUND | Center: ({self.hazard_center_x},{self.hazard_center_y}) | Radius: {self.hazard_radius}m"
        )

    # publisher
        self._pub = self.create_publisher(
            AgvTelemetry,
            "/agv/telemetry",
            10
        )

    # service server
        self._service_validate_goal = self.create_service(
            ValidateWaypoint,
            "/agv/validate_waypoint",
            self.service_validate_goal_callback,
            callback_group = self.cb_group
        )

    # action server
        self._action_navigate_goal = ActionServer(
            self,
            NavigateToWaypoint,
            "/agv/navigate_to_waypoint",
            self.action_navigate_goal_callback,
            callback_group = self.cb_group
        )

    # timer
        self.timer = self.create_timer(
            0.5,
            self.class_timer_callback,
            callback_group = self.cb_group
        )

    def class_timer_callback(self):

        msg = AgvTelemetry()
        msg.current_x = self.current_x
        msg.current_y = self.current_y
        msg.current_battery = self.battery
        msg.status = "IDLE"

        self.get_logger().info(
            f"[TELEMETRY] | Pos: ({msg.current_x:.1f},{msg.current_y:.1f}) | Battery: {msg.current_battery:.1f} | Status: {msg.status} |"
        )
        self._pub.publish(msg)

    def service_validate_goal_callback(self,request,response):

        self.get_logger().info(
            f"[SERVICE] Received waypoint validation request for goal coordinate -> ({request.target_x:.1f},{request.target_y:.1f}) "
        )
        if(self.current_x==request.target_x and self.current_y==request.target_y):
            self.get_logger().warn(
                f"[SERVICE] CURRENT POSITION IS TARGET SITE | Validation DISAPPROVED |"
            )
            response.is_valid = False
            response.estimated_distance = 0.0
            response.reason = "ALREADY ON SITE"
            return response       
        if(self.battery <= 0):
            self.get_logger().warn(
                f"[SERVICE] BATTERY DIED CANNOT GO ANY FURTHER | Validation DISAPPROVED |"
            )
            response.is_valid = False
            response.estimated_distance = 0.0
            response.reason = "BATTERY DIED"
            return response
        
        distance_to_hazard,safe = self.validate_if_safe(request.target_x,request.target_y,self.hazard_center_x,self.hazard_center_y,self.hazard_radius)
        if safe:
            self.get_logger().info(
                f"[SERVICE] Target CLEAR | Distance to hazard : {distance_to_hazard:.1f} | Validation APPROVED |"
            )
            response.is_valid = True
            response.estimated_distance = self.calculate_distance_between_two_points(self.current_x,self.current_y,request.target_x,request.target_y)
            response.reason = "CLEAR"
            return response    
        else:
            self.get_logger().error(
                f"[SERVICE] Target NOT CLEAR | Distance to hazard center : {distance_to_hazard:.1f} | Validation DISAPPROVED |"
            )
            self.get_logger().error(
                f"MISSION ABORTED | Proposed destination is unsafe. Action will not be dispatched |"
            )
            response.is_valid = False
            response.reason = f"Inside Hazard Zone | Target inside the hazard zone"
            return response    

    def action_navigate_goal_callback(self,goal_handle):

        self.get_logger().info(
            f"[Action] Received navigation goal | Target : ({goal_handle.request.target_x:.1f},{goal_handle.request.target_y:.1f}) | Cargo : {goal_handle.request.payload_weight}kg |"
        )
        self.get_logger().info(
            f"[Action] Goal Accepted | beginning trajectory execution..."
        )

        feedback_msg = NavigateToWaypoint.Feedback()

        distance_traveled = 0

        if self.battery <= 0.0:
            self.get_logger().error("[ACTION] MISSION REJECTED | Battery is dead | Cannot perform")
            goal_handle.abort()
            result = NavigateToWaypoint.Result()
            result.reached = False
            result.total_distance_traveled = 0.0
            result.remaining_battery = 0.0
            return result

        while True:
            
            distance_per_unit_time = self.linear_speed * 1 # m/s * s
            distance_remaining = self.calculate_distance_between_two_points(
                self.current_x,self.current_y,
                goal_handle.request.target_x,goal_handle.request.target_y
            )
            if(distance_remaining <= distance_per_unit_time):
                battery_drain_last_moment = self.drain_battery(distance_remaining,goal_handle.request.payload_weight)
                self.battery = self.battery - battery_drain_last_moment

                if self.battery <= 0.0:
                    self.battery = 0.0
                    
                    goal_handle.abort()
                    
                    result = NavigateToWaypoint.Result()
                    result.reached = False
                    result.total_distance_traveled = distance_traveled
                    result.distance_remaining = distance_remaining
                    result.remaining_battery = 0.0
                    self.get_logger().error(
                        f"[ACTION] MISSION ABORTED | Battery is dead, cannot perform task | Distance left : {distance_remaining:.1f}m"
                    )
                    return result
                
                distance_traveled = distance_traveled + distance_remaining
                self.current_x = goal_handle.request.target_x
                self.current_y = goal_handle.request.target_y
                self.get_logger().info(
                    f"[NAVIGATING...] Pos : ({self.current_x:.1f},{self.current_y:.1f}) | Remaining : {distance_remaining:.1f} | Battery : {self.battery:.1f}"
                )
                feedback_msg.distance_to_goal = distance_remaining
                feedback_msg.current_battery = self.battery 
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(1.0)
                break

            delta_x = goal_handle.request.target_x - self.current_x
            delta_y = goal_handle.request.target_y - self.current_y
            battery_drain_per_unit_time = self.drain_battery(distance_per_unit_time,goal_handle.request.payload_weight)
            self.battery = self.battery - battery_drain_per_unit_time
            
            if self.battery <= 0.0:
                self.battery = 0.0
                
                goal_handle.abort()
                
                result = NavigateToWaypoint.Result()
                result.reached = False
                result.total_distance_traveled = distance_traveled
                result.distance_remaining = distance_remaining
                result.remaining_battery = 0.0
                self.get_logger().error(
                    f"[ACTION] MISSION ABORTED | Battery is dead, cannot perform task | Distance left : {distance_remaining:.1f}m"
                )
                return result

            distance_traveled = distance_traveled + distance_per_unit_time

            self.current_x = self.current_x + distance_per_unit_time * (delta_x/distance_remaining)
            self.current_y = self.current_y + distance_per_unit_time * (delta_y/distance_remaining)
            
            feedback_msg.distance_to_goal = distance_remaining
            feedback_msg.current_battery = self.battery 

            self.get_logger().info(
                f"[NAVIGATING...] Pos : ({self.current_x:.1f},{self.current_y:.1f}) | Remaining : {distance_remaining:.1f} | Battery : {self.battery:.1f}"
            )

            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1.0)

        goal_handle.succeed()
        result = NavigateToWaypoint.Result()
        result.reached = True
        result.total_distance_traveled = distance_traveled
        result.remaining_battery = self.battery
        self.get_logger().info(
            "[ACTION] GOAL REACHED | Destination reached "
        )
        return result
    
    # helper function(s) ========================================================================
    def drain_battery(self,distance_per_unit_time,payload_weight):
        """returns the amount of battery consumed per uint of time

        Args:
            distance_per_unit_time (float): distance covered in unit time
            payload_weight (float): weight of the payload from the client

        Returns:
            float: reduce this much battery after every unit time
        """
        return distance_per_unit_time * (self.base_energy_rate + (0.05 * payload_weight))

    def calculate_distance_between_two_points(self,x,y,tx,ty):
        """returns a float value of distance between two 2d coordinates

        Args:
            x (float): current x coordinate
            y (float): current y coordinate
            tx (float): target x coordinate
            ty (float): target y coordinate

        Returns:
            float: distance between two points
        """
        distance = 0.0
        delta_x = tx-x
        delta_y = ty-y
        return math.sqrt((delta_x*delta_x)+(delta_y*delta_y))

    def validate_if_safe(self,tx,ty,hx,hy,hr):
        """returns a float value of distance between target and hazard and a bool value True/False for target is safe or not

        Args:
            tx (float): target x coordinate
            ty (float): target y coordinate
            hx (float): hazard circle x coordinate
            hy (float): hazard circle y coordinate
            hr (float): hazard circle radius

        Returns:
            float: distance between target and nearest hazard
            bool: if target is safe or not
        """
        target_hazard_distance = self.calculate_distance_between_two_points(tx,ty,hx,hy)
        if target_hazard_distance > hr:
            return (target_hazard_distance-hr),True
        else:
            return (target_hazard_distance),False

    # helper function (AI WRITTEN) ==================================================================
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
    node = Agv()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()