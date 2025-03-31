import pyads
import yaml
from ctypes import sizeof
import rclpy
from rclpy.node import Node
from ros2_pyads.ads_com import ADSCom
import sys
from rclpy.node import Node
from pp_library import Pickplace_Driver
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose
from om_aiv_msg.msg import Status
from sensor_msgs.msg import JointState

class MoMaBeckhoffNode(Node):
    """
    A ROS2 node that tests the ADS communication object by writing to a boolean variable in the PLC.
    """

    def __init__(self):
        super().__init__('ads_com_bool_test_node',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        # ROS 2 subscribers
        self.create_subscription(Status, 'ldarcl_status', self.LD_status_callback, 10)
        self.create_subscription(JointState, 'joint_states', self.TM_status_callback, 10)

        self.pickplace_driver = Pickplace_Driver.PickPlaceClass()

        # Flag to ensure variables are initialized before callbacks are called
        self.variables_initialized = False

        # Expected joint names
        self.expected_joint_names = ['joint_1', 'joint_2', 'joint_3', 
                                   'joint_4', 'joint_5', 'joint_6']

        # Get the parameters from the launch file
        com_config = self.get_parameter('com_config').value
        plc_admin = self.get_parameter('plc_admin').value

        if not com_config:
            self.get_logger().fatal('Failed to get "com_config" parameter')
            exit(2)

        if not plc_admin:
            self.get_logger().fatal('Failed to get "plc_admin" parameter')

        # Load the config data from the YAML file
        with open(com_config, 'r') as file:
            com_config_data = yaml.safe_load(file)

        # Load the PLC admin data from the YAML file
        with open(plc_admin, 'r') as file:
            plc_admin_data = yaml.safe_load(file)

        # Initialize the ADS communication object
        self.ads_com = ADSCom(com_config_data, plc_admin_data)

        self.test_write_operations()

        # Set up monitoring for ld execute
        self.setup_ld_execute_callback()

        self.setup_tm_execute_callback()

    def test_write_operations(self):
        """Test writing different variable types."""
        # Test boolean write
        try:
            current_value = self.ads_com.read_by_name('MAIN.bTest', pyads.PLCTYPE_BOOL)
            success = self.ads_com.write_by_name(
                var_name='MAIN.bTest',
                var_value=not current_value,
                var_type=pyads.PLCTYPE_BOOL)

            if success:
                self.get_logger().info('Successfully toggled MAIN.bTest')
            else:
                self.get_logger().error('Failed to write to MAIN.bTest')
        except Exception as e:
            self.get_logger().error(f'Error writing boolean: {str(e)}')

        # Test LREAL write
        try:
            success = self.ads_com.write_by_name(
                var_name='MAIN.lrTest',
                var_value=3.14159,
                var_type=pyads.PLCTYPE_LREAL)

            if success:
                self.get_logger().info('Successfully wrote to MAIN.lrTest')
            else:
                self.get_logger().error('Failed to write to MAIN.lrTest')
        except Exception as e:
            self.get_logger().error(f'Error writing LREAL: {str(e)}')

        test = self.ads_com.read_by_name('MoMa.TM_joint_1_goal', pyads.PLCTYPE_LREAL)
        self.get_logger().info("test.")
        self.get_logger().info(f"Read value {test}")

    def setup_ld_execute_callback(self):
        """Simplified working implementation for pyads 3.3.9"""
        self.tags = {"MoMa.Execute_LD_goal": pyads.PLCTYPE_BOOL}
        attr = pyads.NotificationAttrib(sizeof(pyads.PLCTYPE_BOOL))
        # add_device_notification returns a tuple of notification_handle and
        # user_handle which we just store in handles
        handles = self.ads_com.plc.add_device_notification('MoMa.Execute_LD_goal', attr, self.handle_execute_ld_position)
        self.get_logger().info("LD execute callback set up")

    def setup_tm_execute_callback(self):
        """Simplified working implementation for pyads 3.3.9"""
        self.tags = {"MoMa.Execute_TM_goal": pyads.PLCTYPE_BOOL}
        attr = pyads.NotificationAttrib(sizeof(pyads.PLCTYPE_BOOL))
        # add_device_notification returns a tuple of notification_handle and
        # user_handle which we just store in handles
        handles = self.ads_com.plc.add_device_notification('MoMa.Execute_TM_goal', attr, self.handle_execute_arm_position)
        self.get_logger().info("TM execute callback set up")
        
    def destroy_node(self):
        """Clean shutdown"""
        if hasattr(self, 'monitor_handle'):
            try:
                self.ads_com.plc.del_device_notification(self.monitor_handle)
            except Exception as e:
                self.get_logger().error(f'Cleanup error: {str(e)}')
        super().destroy_node()
        
    def LD_status_callback(self, msg):
        try:
            self.ads_com.write_by_name(
                var_name='MoMa.LD_status',
                var_value=msg.status,
                var_type=pyads.PLCTYPE_STRING)

            self.ads_com.write_by_name(
                var_name='MoMa.LD_pos_x',
                var_value=msg.location.x,
                var_type=pyads.PLCTYPE_LREAL)

            self.ads_com.write_by_name(
                var_name='MoMa.LD_pos_y',
                var_value=msg.location.y,
                var_type=pyads.PLCTYPE_LREAL)

            self.ads_com.write_by_name(
                var_name='MoMa.LD_pos_theta',
                var_value=msg.location.theta,
                var_type=pyads.PLCTYPE_LREAL)

            self.ads_com.write_by_name(
                var_name='MoMa.LD_state_of_charge',
                var_value=msg.state_of_charge,
                var_type=pyads.PLCTYPE_LREAL)

        except Exception as e:
            self.get_logger().error(f'Error writing boolean: {str(e)}')

    def TM_status_callback(self, msg):
        if list(msg.name) == self.expected_joint_names and len(msg.position) >= 6:
            try:
                self.ads_com.write_by_name(
                    var_name='MoMa.TM_joint_1',
                    var_value=msg.position[0],
                    var_type=pyads.PLCTYPE_LREAL)

                self.ads_com.write_by_name(
                    var_name='MoMa.TM_joint_2',
                    var_value=msg.position[1],
                    var_type=pyads.PLCTYPE_LREAL)
            
                self.ads_com.write_by_name(
                    var_name='MoMa.TM_joint_3',
                    var_value=msg.position[2],
                    var_type=pyads.PLCTYPE_LREAL)

                self.ads_com.write_by_name(
                    var_name='MoMa.TM_joint_4',
                    var_value=msg.position[3],
                    var_type=pyads.PLCTYPE_LREAL)

                self.ads_com.write_by_name(
                    var_name='MoMa.TM_joint_5',
                    var_value=msg.position[4],
                    var_type=pyads.PLCTYPE_LREAL)

                self.ads_com.write_by_name(
                    var_name='MoMa.TM_joint_6',
                    var_value=msg.position[5],
                    var_type=pyads.PLCTYPE_LREAL)

            except Exception as e:
                self.get_logger().error(f'Error writing boolean: {str(e)}')
        else:
            # Optional: log when we receive unexpected joint names
            self.get_logger().debug(
                f"Ignoring joint states with names: {msg.name}. "
                f"Expected: {self.expected_joint_names}",
                throttle_duration_sec=5.0  # Only log this every 5 seconds to avoid spam
            )

    # Data change callback for 'ExecuteArmPosition'
    def handle_execute_arm_position(self, notification, data):
        """Callback function for when ExecuteArmPosition changes"""
        data_type = self.tags[data]
        handle, timestamp, value = self.ads_com.plc.parse_notification(notification, data_type)
        self.get_logger().info(f"LD execute changed to {bool(value)}")
        if value:
            self.get_logger().info("Execute Arm Position set to True.")

            try:
                # current_value = [self.ads_com.read_by_name('MoMa.TM_joint_1_goal', pyads.PLCTYPE_LREAL), self.ads_com.read_by_name('MoMa.TM_joint_2_goal', pyads.PLCTYPE_LREAL), self.ads_com.read_by_name('MoMa.TM_joint_3_goal', pyads.PLCTYPE_LREAL), self.ads_com.read_by_name('MoMa.TM_joint_4_goal', pyads.PLCTYPE_LREAL), self.ads_com.read_by_name('MoMa.TM_joint_5_goal', pyads.PLCTYPE_LREAL), self.ads_com.read_by_name('MoMa.TM_joint_6_goal', pyads.PLCTYPE_LREAL)]
                # self.get_logger().info(
                #     f"Goal Arm Pose: [{current_value[0]:.4f}, {current_value[1]:.4f}, "
                #     f"{current_value[2]:.4f}, {current_value[3]:.4f}, "
                #     f"{current_value[4]:.4f}, {current_value[5]:.4f}]"
                # )
                self.test_write_operations()
                self.get_logger().info("test.")
                test = self.ads_com.read_by_name('MoMa.TM_joint_1_goal', pyads.PLCTYPE_LREAL)
                self.get_logger().info("test.")
                self.get_logger().info(f"Read value {test}")
                # Reset the execute flag
                self.ads_com.write_by_name(
                    var_name='MoMa.Execute_TM_goal',
                    var_value=False,
                    var_type=pyads.PLCTYPE_BOOL)
                
                # Here you would call your pickplace_driver if needed
                # self.pickplace_driver.set_position(current_value)
                
            except Exception as e:
                self.get_logger().error(f"Error handling arm position: {str(e)}")
        
        else:
            self.get_logger().info("Execute Arm Position set to False.")

    # Data change callback for 'ExecuteLDPosition'
    def handle_execute_ld_position(self, notification, data):
        data_type = self.tags[data]
        handle, timestamp, value = self.ads_com.plc.parse_notification(notification, data_type)
        self.get_logger().info(f"LD execute changed to {bool(value)}")
        """Callback function for when ExecuteLDPosition changes"""
        if value:
            self.get_logger().info("Execute LD Position set to True.")
        else:
            self.get_logger().info("Execute LD Position set to False.")

def main(args=None):
    rclpy.init(args=args)
    node = MoMaBeckhoffNode()
    try:
        rclpy.spin(node)  # Run ROS 2 node in the main thread
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()