'''robot_node.py | [pub] [service ser] [action ser] |

PARAMETERS: 
    - robot_name : (Robby)
    - drain_rate : (5)

PUBLISHER:
    - timer : 1.0
    - if not charging dec battery_level
    - PUBLISHES : BatteryStatus | TOPIC : battery_status

SERVICE SERVER:  
    - TYPE : ResetBattery | TOPIC : reset_battery
    - sets battery_level = request.new_level

ACTION SERVER:
    - TYPE : ChargeBattery | TOPIC : charge_battery
    - marks is_charging = True
    - FEEDBACK : inc 5% per sec until target_level
    - marks is_charging = False
'''

from time import sleep

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from rcl_interfaces.msg import ParameterDescriptor,SetParametersResult

from robot_interface.msg import BatteryStatus
from robot_interface.srv import ResetBattery
from robot_interface.action import ChargeBattery


class Robot(Node):

    def __init__(self):

    # node name
        super().__init__("robot_app")

    # value variables
        self.battery_level = 100
        self.is_charging = False

    # parameters
        self.declare_parameter(
            "robot_name",
            "Robby",
            ParameterDescriptor(description = "sets robot name (string) [default : Robby]")
        )
        self.robot_name = self.get_parameter("robot_name").value
        self.declare_parameter(
            "drain_rate",
            5,
            ParameterDescriptor(description = "Sets drain rate of battery (int) [default : 5] [1-99]")
        )
        self.drain_rate = self.get_parameter("drain_rate").value

    # parameter caller
        self.add_on_set_parameters_callback(self.parameter_change_callback)

    # node starting info
        self.get_logger().info(f"NODE STARTED | ROBOT : {self.robot_name} | DRAIN RATE : {self.drain_rate} | BATTERY LEVEL : {self.battery_level} |")

    # publisher
        self.publisher_ = self.create_publisher(
            BatteryStatus,
            'battery_status',
            10
        )

    # service server
        self.server_service_ = self.create_service(
            ResetBattery,
            'reset_battery',
            self.reset_battery_callback
        )

    # action server
        self.action_service_ = ActionServer(
            self,
            ChargeBattery,
            'charge_battery',
            self.charge_battery_callback
        )

    # timer
        self.timer = self.create_timer(
            1.0,
            self.class_callback
        )

    # publisher part
    def class_callback(self):

        msg = BatteryStatus()
        msg.robot_name = self.robot_name
        msg.battery_level = self.battery_level
        msg.is_charging = self.is_charging

        self.get_logger().info(f"PUBLISHING  | BATTERY : {self.battery_level} | CHARGING : {self.is_charging} |")
        self.publisher_.publish(msg)

        if self.is_charging == False:
            self.battery_level -= self.drain_rate
            if self.battery_level <= 0:
                self.battery_level = 0
                self.get_logger().warn("TERMINATING | BATTERY FULLY DRAINED SHUTTING DOWN |")
                self.timer.cancel()

    # service server part
    def reset_battery_callback(self,request,response):

        self.get_logger().info(f"SERVICE REQUEST RECEIVED: Reset battery to {request.new_level}%")

        response.success = True
        response.message = f"BATTERY SET TO {request.new_level}%"

        self.battery_level = request.new_level

        self.get_logger().info("[SUCCESS] battery reset successful")

        return response

    # action server part
    def charge_battery_callback(self,goal_handle):
        
        self.get_logger().info(f"ACTION RECEIVED : charge goal to {goal_handle.request.target_level}")

        if self.battery_level>=goal_handle.request.target_level:
            result = ChargeBattery.Result()
            result.success = False
            return result

        feedback_msg = ChargeBattery.Feedback()
        feedback_msg.current_level = self.battery_level

        self.is_charging = True

        while(feedback_msg.current_level < goal_handle.request.target_level):
            if feedback_msg.current_level+5 > goal_handle.request.target_level:
                feedback_msg.current_level = goal_handle.request.target_level
            else :
                feedback_msg.current_level += 5
            self.get_logger().info(f"CHARGING... current battery level : {feedback_msg.current_level}")
            goal_handle.publish_feedback(feedback_msg)
            sleep(1.0)

        self.is_charging = False

        goal_handle.succeed()
        self.battery_level = feedback_msg.current_level
        self.get_logger().info("GOAL SUCCEEDED : battery level reached target level")

        result = ChargeBattery.Result()
        result.final_level = feedback_msg.current_level
        result.success = True

        return result

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
    node = Robot()    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()