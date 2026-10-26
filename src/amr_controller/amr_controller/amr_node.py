'''arm_node [pub] [sub] [service server] [action ser]

# DEFINED VALUE(s)
    - current_x = 0.0 (m)
    - current_y = 0.0 (m)
    - current_theta = 0.0 (rad)
    - current_linear_velocity = 0.0 (m/s)
    - current_angular_velocity = 0.0 (rad/s)

PARAMETER:
    - linear_velocity_value = 0.6 [0.1 - 1.5]
    - angular_velocity_value = 1.2 [0.2 - 3.14]
    - angular_velocity_multiplayer = 1.5 [0.5 - 5.0]

PUBLISHER:
    - TYPE: Pose2DStamped | TOPIC: /amr/odometry
    - MSG:
        - x : current_x
        - y : current_y
        - theta : current_theta
        - linear_velocity : current_linear_velocity
        - angular_velocity : current_angular_velocity
    - Gives out current dash-board information
        
SUBSCRIBER: (i dont get why we need to use this tbh so i am leaving this)
    - TYPE: SpeedLimit | TOPIC: /amr/speed_limit
    - MSG:
        - max_linear_speed : current_max_linear_speed
    - It speeds up or slows the max speed (near the goal site in our case)
    - using in speed formula

SERVICE SERVER:
    - TYPE: EmergencyBreak | TOPIC: /amr/emergency_break
    - SRV:
        - REQUEST:
            - engage_break : bool
        - RESPONSE:
            - success : bool
            - message : string
    - is triggered by a client that depends on the initial mode of the supervisor
    - we just put velocity = 0 and stays there

ACTION SERVER:
    - TYPE: NavigateToPose.action | TOPIC: /amr/navigate_to_pose
    - ACTION:
        - GOAL:
            - goal_x : x coordinate of goal
            - goal_y : y coordinate of goal
            - tolerance : range near the goal where we can assume we reached
        - RESULT: 
            - reached : bool
            - final_x : x under tolerance range
            - final_y : y under tolerance range
            - final_theta : theta under tolerance range
            - termination_reason : in case we dont reach out goal the reason (can be breaking)
        - FEEDBACK:
            - current_x : current_x
            - current_y : current_y
            - current_theta : current_theta
            - distance remaining : distance_remaining
'''

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult, FloatingPointRange

from amr_interface.msg import Pose2DStamped, SpeedLimit
from amr_interface.srv import EmergencyBreak
from amr_interface.action import NavigateToPose

class Amr(Node):

    def __init__(self):

    #* node name: amr_node
        super().__init__("amr_node")

        self.cb_group = ReentrantCallbackGroup()

    #* parameters
        self.add_on_set_parameters_callback(self.parameter_change_callback)

        self.declare_bounded_param( "linear_velocity_value" , 0.6 , 0.1 , 1.5 )
        self.declare_bounded_param( "angular_velocity_value" , 1.2 , 0.2 , 3.14 )
        self.declare_bounded_param( "angular_velocity_multiplayer" , 1.5 , 0.5 , 5.0 )

    #* defined value(s)
        self.current_x = 0.0 # (m)
        self.current_y = 0.0 # (m)
        self.current_theta = 0.0 # (rad)
        self.current_linear_velocity = 0.0 # (m/s)
        self.current_angular_velocity = 0.0  # (rad/s)

    #* node starting message
        self.get_logger().info(
            f"AMR ONLINE | Pose : ({self.current_x} , {self.current_y}) , {self.current_theta})"
        )

    #* publisher
        self._pub_odometer = self.create_publisher(
            Pose2DStamped,
            "/amr/odometry",
            10
        )
        self._class_timer = self.create_timer(
            0.10,
            self._class_timer_callback, 
            callback_group=self.cb_group
        )

    #* service server
        self._ser_emergency_break_ser = self.create_service(
            EmergencyBreak,
            "/amr/emergency_break",
            self.emergency_break_ser_callback
        )

    #* action server
        self._act_navigate_goal_ser = ActionServer(
            self,
            NavigateToPose,
            "/amr/navigate_to_pose",
            self._act_navigate_goal_ser_callback,
            callback_group=self.cb_group
        )


# ===================================== timer function (0.1 sec) =====================================
    def _class_timer_callback(self):

        msg = Pose2DStamped()
        msg.x = self.current_x
        msg.y = self.current_y
        msg.theta = self.current_theta
        msg.linear_velocity = self.current_linear_velocity 
        msg.angular_velocity = self.current_angular_velocity

        self.get_logger().info(
            f"[PUB] Pose : ({self.current_x} , {self.current_y} , {self.current_theta}) | v : {self.current_linear_velocity} w : {self.current_angular_velocity}"
        )

        self._pub_odometer.publish(msg)        

# =================================== service ser (BREAK | v = 0) ====================================
    def emergency_break_ser_callback(self,request,response):

        pass

# ====================================== action ser (navigate) =======================================
    def _act_navigate_goal_ser_callback(self,goal_handle):

        goal_x = goal_handle.request.goal_x
        goal_y = goal_handle.request.goal_y
        tolerance = goal_handle.request.tolerance

        self.get_logger().info(
            f"[ACT SER] Received nav goal | Goal pose : {goal_x,goal_y} | Tolerance : {tolerance}"
        )
        if self.goal_distance_current(self.current_x,self.current_y,goal_x,goal_y) < tolerance:
            self.get_logger().error(
                f"[ACT SER] GOAL REJECTED | Target already reached"
            )
            goal_handle.succeed()
            result = NavigateToPose.Result()
            result.reached = True
            result.final_x = self.current_x
            result.final_y = self.current_y
            result.final_theta = self.current_theta
            return result

        while True:

            distance_remaining = self.goal_distance_current(self.current_x,self.current_y,goal_x,goal_y)
            if distance_remaining <= tolerance:
                goal_handle.succeed()
                result = NavigateToPose.Result()
                result.reached = True
                result.final_x = self.current_x
                result.final_y = self.current_y
                result.final_theta = self.current_theta
                result.termination_reason = "GOAL_REACHED"
                self.get_logger().info(
                    f"[ACT SER] MISSION COMPLETED | Goal reached"
                )
                self.current_linear_velocity = 0.0
                self.current_angular_velocity = 0.0
                return result

            _line_of_sight_angle = self.line_of_sight_angle(self.current_x,self.current_y,goal_x,goal_y)
            _goal_relative_current_angle = self.goal_relative_current_angle(self.current_theta,_line_of_sight_angle)
            _angular_kin_controller_output = self.angular_kinematic_controller(self.angular_velocity_multiplayer,_goal_relative_current_angle,self.angular_velocity_value)
            _linear_kin_controller_output = self.linear_kinematic_controller(self.linear_velocity_value,_goal_relative_current_angle)
            self.update_state(_linear_kin_controller_output,_angular_kin_controller_output)

            self.current_linear_velocity = _linear_kin_controller_output
            self.current_angular_velocity = _angular_kin_controller_output

            feedback_msg = NavigateToPose.Feedback()
            feedback_msg.current_x = self.current_x
            feedback_msg.current_y = self.current_y
            feedback_msg.current_theta = self.current_theta
            feedback_msg.distance_remaining = distance_remaining
            
            goal_handle.publish_feedback(feedback_msg)

            time.sleep(0.1)

# ================================= calculation functions (navigate) =================================

    def goal_distance_current(self,x,y,gx,gy):
        """gives distance between current position and goal position

        Args:
            x (float): x coordinate of current location
            y (float): y coordinate of current location
            gx (float): x coordinate of goal
            gy (float): y coordinate of goal
        
        Returns:
            float: distance between goal and current position
        """
        delta_x = gx-x
        delta_y = gy-y
        distance = math.sqrt((delta_x*delta_x) + (delta_y*delta_y))
        return distance

    def line_of_sight_angle(self,x,y,gx,gy):
        """gives the angle from current location to goal angle

        Args:
            x (float): x coordinate of current location
            y (float): y coordinate of current location
            gx (float): x coordinate of goal
            gy (float): y coordinate of goal
        
        Returns:
            float: the angle where the goal is from our current location
        """
        delta_x = gx-x
        delta_y = gy-y
        angle = math.atan2(delta_y,delta_x)
        return angle

    def goal_relative_current_angle(self,current_theta,goal_theta):
        """gives the relative angle between current position and goal

        Args:
            current_theta (float): (rad) current angle of state
            goal_theta (float): (rad) angle of goal from current state

        Returns:
            float: (rad) relative angle from current state to goal
        """
        relative_theta = math.atan2(
            math.sin(goal_theta - current_theta),
            math.cos(goal_theta - current_theta)
        )
        return relative_theta

    def clamp(self, val, min_val, max_val):
        return max(min_val, min(val, max_val))

    def angular_kinematic_controller(self,w_velo_multiplier,relative_angle,angular_velo_value):
        """the angular velo that needs to be kept while moving in certain angle

        Args:
            w_velo_multiplier (float): angular_velocity_multiplier from parameter
            relative_angle (float): (rad) the relative angle between goal and current position
            angular_velo (float): (rad/m) the value of angular_velocity_value parameter
    
        Returns:
            float: (rad/m) the angular speed that needs to be maintained at the moment given the position
        """
        angular_velocity = self.clamp( (w_velo_multiplier*relative_angle) , -angular_velo_value , angular_velo_value )
        return angular_velocity

    def linear_kinematic_controller(self,linear_velo_value,relative_angle):
        """the linear velo that needs to be kept while moving towards the goal

        Args:
            linear_velo_value (float): linear_velocity_value parameter
            relative_angle (float): (rad) the relative angle from current position to goal

        Returns:
            float: the linear speed that need to kept to reach the goal from current position
        """
        linear_velocity = linear_velo_value * max(0.0 , math.cos(relative_angle))
        return linear_velocity

    def update_state(self, linear_velo, angular_velo, delta_time = 0.1):
        """updates the current state by the calculated values to the rover to move

        Args:
            linear_velo (float): the value of linear_kinematic_controller
            angular_velo (float): the value of angular_kinematic_controller
            delta_time (float, optional): time difference. Defaults to 0.1.
        """
        self.current_x += linear_velo * math.cos(self.current_theta) * delta_time
        self.current_y += linear_velo * math.sin(self.current_theta) * delta_time

        current_theta_raw = self.current_theta + (angular_velo * delta_time)
        self.current_theta = math.atan2(
            math.sin(current_theta_raw),
            math.cos(current_theta_raw)
        ) 

# ======================================= helper function (AI) ========angular_kinematic_controller===============================
    def declare_bounded_param(self, name, default, min_val, max_val):
        """AI WRITTEN FUNCTION : creates a parameter with bound 
    
        Args:
            name (string): name of the parameter
            default (anything): default value of the parameter
            min_val (anything): minimum value of the parameter
            max_val (anything): maximum value of the parameter

        Returns:
            a parameter that can be directly used by the name of parameter itself
        """
        limit = FloatingPointRange(
            from_value=float(min_val), to_value=float(max_val)
        )
        desc = ParameterDescriptor(floating_point_range=[limit])

        self.declare_parameter(name, float(default), descriptor=desc)

        setattr(self, name, default)

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
    node = Amr()
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