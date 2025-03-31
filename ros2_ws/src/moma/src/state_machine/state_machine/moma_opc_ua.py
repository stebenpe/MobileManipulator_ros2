import asyncio
import threading
import sys
from asyncua import Server, ua
import rclpy
from rclpy.node import Node
from pp_library import Pickplace_Driver
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose
from om_aiv_msg.msg import Status
from sensor_msgs.msg import JointState

class OPCUAServerNode(Node):
    def __init__(self):
        super().__init__('opcua_server_node')

        self.server = Server()
        self.server.set_endpoint("opc.tcp://192.168.44.13:4840/freeopcua/server/")
        
        # ROS 2 subscribers
        self.create_subscription(Status, 'ldarcl_status', self.LD_status_callback, 10)
        self.create_subscription(JointState, 'joint_states', self.TM_status_callback, 10)

        self.pickplace_driver = Pickplace_Driver.PickPlaceClass()

        # Flag to ensure variables are initialized before callbacks are called
        self.variables_initialized = False

        # Expected joint names
        self.expected_joint_names = ['joint_1', 'joint_2', 'joint_3', 
                                   'joint_4', 'joint_5', 'joint_6']

        # Start the OPC UA server in a background thread
        self.opcua_thread = threading.Thread(target=self.run_opcua_server, daemon=True)
        self.opcua_thread.start()

    def LD_status_callback(self, msg):
        # Await the asynchronous set_value calls using asyncio.run_coroutine_threadsafe
        if self.variables_initialized:
            # msg = Status()
            asyncio.run_coroutine_threadsafe(self.ld_status.set_value(msg.status), self.loop)
            asyncio.run_coroutine_threadsafe(self.ld_position.set_value([msg.location.x, msg.location.y, msg.location.theta]), self.loop)
            asyncio.run_coroutine_threadsafe(self.ld_state_of_charge.set_value(msg.state_of_charge), self.loop)

    def TM_status_callback(self, msg):
        if not self.variables_initialized:
            return
            
        # Check if the message contains exactly our expected joints in order
        if list(msg.name) == self.expected_joint_names and len(msg.position) >= 6:
            try:
                joint_positions = [float(pos) for pos in msg.position[:6]]
                asyncio.run_coroutine_threadsafe(
                    self.robot_arm_pose.set_value(joint_positions), 
                    self.loop
                )
            except (ValueError, IndexError) as e:
                self.get_logger().warn(f"Error processing joint states: {str(e)}")
        else:
            # Optional: log when we receive unexpected joint names
            self.get_logger().debug(
                f"Ignoring joint states with names: {msg.name}. "
                f"Expected: {self.expected_joint_names}",
                throttle_duration_sec=5.0  # Only log this every 5 seconds to avoid spam
            )

    # Data change callback for 'ExecuteArmPosition'
    def handle_execute_arm_position(self, node, val, data):
        """Callback function for when ExecuteArmPosition changes"""
        if val:
            self.get_logger().info("Execute Arm Position set to True.")
        
            # Create a coroutine to get the value and log it
            async def get_and_log_value():
                try:
                    current_value = await self.goal_arm_pose.get_value()
                    self.get_logger().info(
                        f"Goal Arm Pose: [{current_value[0]:.4f}, {current_value[1]:.4f}, "
                        f"{current_value[2]:.4f}, {current_value[3]:.4f}, "
                        f"{current_value[4]:.4f}, {current_value[5]:.4f}]"
                    )
                
                    # Reset the execute flag
                    await self.execute_arm_position.set_value(False)
                
                    # Here you would call your pickplace_driver if needed
                    self.pickplace_driver.set_position(current_value)
                
                except Exception as e:
                    self.get_logger().error(f"Error handling arm position: {str(e)}")
        
            # Run the coroutine in the event loop
            asyncio.run_coroutine_threadsafe(get_and_log_value(), self.loop)
        
        else:
            self.get_logger().info("Execute Arm Position set to False.")

    # Data change callback for 'ExecuteLDPosition'
    def handle_execute_ld_position(self, node, val, data):
        """Callback function for when ExecuteLDPosition changes"""
        if val:
            self.get_logger().info("Execute LD Position set to True.")
        else:
            self.get_logger().info("Execute LD Position set to False.")

    def run_opcua_server(self):
        asyncio.run(self.start_opcua_server())

    async def start_opcua_server(self):
        # Initialize the server
        await self.server.init()

        # Set up a namespace
        uri = "urn:ros2:opcua"
        idx = await self.server.register_namespace(uri)

        # Example 1: Create basic objects
        self.robot_arm_pose = await self.server.nodes.objects.add_variable(idx, "RobotArmPose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.ld_position = await self.server.nodes.objects.add_variable(idx, "LDPosition", [0.0, 0.0, 0.0])
        self.ld_status = await self.server.nodes.objects.add_variable(idx, "LDStatus", "Idle")
        self.ld_state_of_charge = await self.server.nodes.objects.add_variable(idx, "LDStateOfCharge", 0.0)
        self.goal_position_name = await self.server.nodes.objects.add_variable(idx, "GoalPositionName", "Goal1")
        self.goal_arm_pose = await self.server.nodes.objects.add_variable(idx, "GoalArmPose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.execute_arm_position = await self.server.nodes.objects.add_variable(idx, "ExecuteArmPosition", False)
        self.execute_ld_position = await self.server.nodes.objects.add_variable(idx, "ExecuteLDPosition", False)

        # Make variables writable
        await self.robot_arm_pose.set_writable()
        await self.ld_position.set_writable()
        await self.ld_status.set_writable()
        await self.ld_state_of_charge.set_writable()
        await self.goal_position_name.set_writable()
        await self.goal_arm_pose.set_writable()
        await self.execute_arm_position.set_writable()
        await self.execute_ld_position.set_writable()

        # Create a subscription for monitoring value changes
        subscription = await self.server.create_subscription(100, self)

        # Add items to the subscription
        await subscription.subscribe_data_change(self.execute_arm_position)
        await subscription.subscribe_data_change(self.execute_ld_position)

        # Set the flag to indicate that the variables are initialized
        self.variables_initialized = True

        # Store the event loop for executing async tasks from synchronous code
        self.loop = asyncio.get_event_loop()

        # Start the server
        async with self.server:
            while rclpy.ok():
                await asyncio.sleep(1)

    def datachange_notification(self, node, val, data):
        """Callback function for data change notifications"""
        # Check which variable changed and handle accordingly
        if node == self.execute_arm_position:
            self.handle_execute_arm_position(node, val, data)
        elif node == self.execute_ld_position:
            self.handle_execute_ld_position(node, val, data)

def main(args=None):
    rclpy.init(args=args)
    node = OPCUAServerNode()

    try:
        rclpy.spin(node)  # Run ROS 2 node in the main thread
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()