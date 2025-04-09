#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from topics_services.msg import Telemetric
from pp_library import Modbus
from om_aiv_msg.msg import Status
from om_aiv_navigation.goto_goal import AmrActionClient
from pp_library import Pickplace_Driver
from pickplace_msgs.srv import AskModbus
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient

class WebsiteIntegrationNode(Node):

    def __init__(self):
        super().__init__("website_integration_node")
        self.publisher_ = self.create_publisher(Telemetric, '/Telemetric', 10)
        self.subscription = self.create_subscription(Status, '/ldarcl_status', self.ld_status, 10)
        self.gripper_action_client = ActionClient(self, GripperCommand, '/robotiq_gripper_controller/gripper_cmd')
        self.get_logger().info("website_integration_node has started")
        self.timer_ = self.create_timer(1, self.SendTelemetric)
        self.modbus = Modbus.ModbusClass()
        self.action_client = AmrActionClient()
        self.pickplace_driver = Pickplace_Driver.PickPlaceClass()
        self.cli = self.create_client(AskModbus, 'ask_modbus')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            print("Service not available...")
        self.req = AskModbus.Request()
        # initialize variables
        self.status = 'unknown'
        self.ld_charge = 0.0
        self.ld_temperature = 0.0
        self.ld_x = 0.0
        self.ld_y = 0.0
        self.ld_theta = 0.0
        self.set_gripper(0.1)  # Open gripper
    
    def SendTelemetric(self):
        msg = Telemetric()
        msg.power = self.modbus.get_power_draw()
        msg.status = self.status
        msg.charge = self.ld_charge
        msg.temperature = self.ld_temperature
        msg.coordinates_x = self.ld_x
        msg.coordinates_y = self.ld_y
        msg.theta = self.ld_theta
        self.publisher_.publish(msg)

    def ld_status(self, msg):
        self.status = msg.status
        self.ld_charge = msg.state_of_charge
        self.ld_temperature = msg.temperature
        self.ld_x = msg.location.x
        self.ld_y = msg.location.y
        self.ld_theta = msg.location.theta

    def send_to_goal(self, goal):
        max_attempts = 5  # Maximum number of tries
        attempts = 0  # Track the number of attempts

        # Loop until goal is achieved or max attempts are exhausted
        while attempts < max_attempts:
            attempts += 1
            goalresult = self.action_client.send_goal(goal)  # Send the goal
            
            if "Arrived at" in goalresult:
                self.get_logger().info(f"Goal achieved on attempt {attempts}/{max_attempts}!")
                break  # Exit loop on success
            
            # Log failure and retry
            self.get_logger().warn(f"Failed to arrive at goal (Attempt {attempts}/{max_attempts})")
            if attempts < max_attempts:
                self.get_logger().info("Retrying in 1 second...")
                time.sleep(1)
        else:
            # This block runs if the loop didn't break (all attempts failed)
            self.get_logger().error(f"Failed to reach goal after {max_attempts} attempts. Aborting.")
            return False
        return True
    
    def set_position(self, position):
        # Set the TM to move to the designated home position
        current_position = self.get_current_pos(self.cli)
        if not self.check_same_positions(current_position, position):
            self.pickplace_driver.set_position(position)

    def get_current_pos(self, cli):
        while not cli.wait_for_service(timeout_sec=1.0):
            print("Service not available...")
            req = AskModbus.Request()
            req.req = 'get_pos'
            future = cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            return future.result().position
        
    def check_same_positions(self, current, goal):
        if (round(current[0]/1000, 3) == round(goal[0],3) and
            round(current[1]/1000, 3) == round(goal[1],3) and
            round(current[2]/1000, 3) == round(goal[2],3) and
            round(current[3]/57.2958, 3) == round(goal[3],3) and
            round(current[4]/57.2958, 3) == round(goal[4],3) and
            round(current[5]/57.2958, 3) == round(goal[5],3)):
                return True
        return False
    
    def set_gripper(self, position):
        # Wait for action server
        if not self.gripper_action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Action server not available!")
            return False
        
        # Create goal message
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = -1.0  # No limit on effort

        # Send goal and wait for result
        try:
            goal_future = self.gripper_action_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, goal_future)
            goal_handle = goal_future.result()
            
            if not goal_handle.accepted:
                self.get_logger().error("Goal rejected by server")
                return False
                
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result = result_future.result().result
            
            self.get_logger().info(
                f"Gripper moved to position: {position} "
                f"(Actual position: {result.position})"
            )
            return True
            
        except Exception as e:
            self.get_logger().error(f"Action failed: {str(e)}")
            return False


def main(args=None):
    rclpy.init(args=args)
    node = WebsiteIntegrationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
   main()