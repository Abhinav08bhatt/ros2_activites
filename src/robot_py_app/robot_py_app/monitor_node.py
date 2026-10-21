'''monitor_node.py | [sub] [service cli] [action cli]

PARAMETERS: 
    - charger_available : (1) (1: True | 0: False)
    - reset_value : (30) 
    - target_level : (75) 

SUBSCRIBER:
    - TYPE : BatteryStatus | TOPIC : battery_status
    - logs incoming msg and warn when charge under 25

SERVICE CLIENT:
    - TYPE : ResetBattery | TOPIC : reset_battery
    - kick : after 5 sec of startup of node -> reset to 30% : 

ACTION CLIENT:
    - TOPIC : charge_battery
    - kick : reset is confirmed -> target_level: 75
    - feedback +5 level every sec
'''

from time import sleep

import rclpy 
from rclpy.node import Node
from rclpy.action import ActionClient

from rcl_interfaces.msg import ParameterDescriptor,SetParametersResult

from robot_interface.msg import BatteryStatus
from robot_interface.srv import ResetBattery
from robot_interface.action import ChargeBattery

class Monitor(Node):

    def __init__(self):

    # node name
        super().__init__("monitor_node")

    # parameter
        self.declare_parameter(
            "charger_available",
            1,
            ParameterDescriptor(description = '(int) [1 : charger available | 0 : charger not available]')
        )
        self.charger_available = self.get_parameter("charger_available").value
        self.declare_parameter(
            "reset_level",
            30,
            ParameterDescriptor(description = 'Sets reset service value (int) [default : 30] [1-100]')
        )
        self.reset_level = self.get_parameter("reset_level").value
        self.declare_parameter(
            "target_level",
            75,
            ParameterDescriptor(description = 'Sets recharge action target level (int) [default : 75] [reset_level-100]')
        )
        self.target_level = self.get_parameter("target_level").value

    # parameter caller
        self.add_on_set_parameters_callback(self.parameter_change_callback)

    # node starting info
        self.get_logger().info(f"NODE STARTED | CHARGER AVAILABLE : {self.charger_available==1} | RESET LEVEL : {self.reset_level} | TARGET LEVEL : {self.target_level} |")

    # subscriber
        self.subscriber_ = self.create_subscription(
            BatteryStatus,
            'battery_status',
            self.subscriber_callback,
            10
        )

    # service client
        self.service_client_ = self.create_client(
            ResetBattery,
            'reset_battery'
        )
        self.client_trigger_timer = self.create_timer(5.0,self.service_client_callback)


    # action
        self.action_client_ = ActionClient(
            self,
            ChargeBattery,
            'charge_battery'
        )


    def subscriber_callback(self,msg):
        '''the subscriber function : 
        logs the info given by the publisher
        '''

        if msg.battery_level > 25:
            self.get_logger().info(f"RECEIVING | Battery status from {msg.robot_name} : {msg.battery_level} |")
        else:
            self.get_logger().warn(f"RECEIVING | [LOW BATTERY] Battery status from {msg.robot_name} : {msg.battery_level} |")

    def service_client_callback(self):
        '''the client function :
        triggered 5.0 sec after the startup of the node (once only)
        it sends a request to reset the charge to the service server
        '''

        self.client_trigger_timer.cancel()

        self.get_logger().info(f"SENDING RESET REQUEST : Set battery to {self.reset_level}%... ")

        if not self.service_client_.wait_for_service(timeout_sec = 5.0):
            self.get_logger().error("SERVICE NOT AVAILABLE (waited for 5.0 second)")
            return

        request = ResetBattery.Request()
        request.new_level = self.reset_level

        future = self.service_client_.call_async(request)
        future.add_done_callback(self.service_client_response_callback)

    def service_client_response_callback(self,future):
        ''' get triggers when the service server sends a response
        job is to handle success or failure of the request
        '''

        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"SERVICE RESPONSE [SUCCESS] : {response.message}")
            
            if self.charger_available == 0:
                self.get_logger().error("CHARGER NOT AVAILABLE : need manual fixing")
            else:
                self.action_client_charge_goal(self.target_level)
            
        except Exception as e:
                self.get_logger().error(f"SERVICE FAILED : {e}")


    def action_client_charge_goal(self,target_level):
        '''get triggered when the charge level is below target level after the recharge
        handles sending goal to server and calling response, feedback, result
        '''
        goal_msg = ChargeBattery.Goal()
        goal_msg.target_level = target_level

        self.action_client_.wait_for_server()

        self.self_goal_future = self.action_client_.send_goal_async(
            goal_msg,
            feedback_callback= self.action_client_charger_feedback    
        )
        self.get_logger().info(f"ACTION GOAL : Sending charge goal to reach {target_level}%...")

        self.self_goal_future.add_done_callback(self.action_client_charger_response)

    def action_client_charger_response(self,future):
        goal_handel = future.result()
        if not goal_handel.accepted:
            self.get_logger().fatal("GOAL REJECTED | need manual fix")
            return

        self._future_result = goal_handel.get_result_async()
        self._future_result.add_done_callback(self.action_client_charger_result)

    def action_client_charger_feedback(self,feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"[FEEDBACK] Current charge level : {feedback.current_level}% | Charging...")

    def action_client_charger_result(self,future):
        result = future.result().result
        if result.success:
            self.get_logger().info(f"[RESULT] Charging completed | Final charge level : {result.final_level}% ")
            return
        else:
            self.get_logger().warn("[RESULT] Battery level already satisfied | Not charged")
        return

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
    node = Monitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()