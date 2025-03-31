import pyads
import yaml
from ctypes import sizeof
import rclpy
from rclpy.node import Node
from ros2_pyads.ads_com import ADSCom


class ADSComBoolTestNode(Node):
    """
    A ROS2 node that tests the ADS communication object by writing to a boolean variable in the PLC.
    """

    def __init__(self):
        super().__init__('ads_com_bool_test_node',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

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

        # Test writing operations
        self.test_write_operations()

        self.get_logger().info(f"Using pyads version: {pyads.__version__}")

        # Set up monitoring for MAIN.mTest
        self.setup_bool_monitoring()

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

    def bool_callback(self, notification, data):
        data_type = self.tags[data]
        handle, timestamp, value = self.ads_com.plc.parse_notification(notification, data_type)
        self.get_logger().info(f"changed to {bool(value)}")

    def setup_bool_monitoring(self):
        """Simplified working implementation for pyads 3.3.9"""
        self.tags = {"MAIN.mTest": pyads.PLCTYPE_BOOL}
        attr = pyads.NotificationAttrib(sizeof(pyads.PLCTYPE_BOOL))
        # add_device_notification returns a tuple of notification_handle and
        # user_handle which we just store in handles
        handles = self.ads_com.plc.add_device_notification('MAIN.mTest', attr, self.bool_callback)
        self.get_logger().info("Callback set up")
        

    def destroy_node(self):
        """Clean shutdown"""
        if hasattr(self, 'monitor_handle'):
            try:
                self.ads_com.plc.del_device_notification(self.monitor_handle)
            except Exception as e:
                self.get_logger().error(f'Cleanup error: {str(e)}')
        super().destroy_node()

def main():
    rclpy.init()
    node = ADSComBoolTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
