import pyads
import yaml
import rclpy
from rclpy.node import Node
from ros2_pyads.ads_com import ADSCom
from ros2_pyads_interfaces.srv import ReadBool, WriteBool, ReadString, ReadByteArray, GetAdsComConfig, WriteInt, MonitorBool, MonitorInt, MonitorReal, MonitorString, MonitorByteArray


class ADSComNode(Node):
    """
    A ROS2 node that tests the ADS communication object by writing to a boolean variable in the PLC.
    """

    def __init__(self):
        super().__init__('ads_com_node',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        # Get the parameters from the launch file
        ads_com_topic = self.get_parameter('ads_config_topic').value
        robot_station = self.get_parameter('robot_station').value
        com_config = self.get_parameter('com_config').value
        plc_admin = self.get_parameter('plc_admin').value

        # Initialize ADS communication
        self._initialize_ads_com(ads_com_topic, robot_station, com_config, plc_admin)

        # Connection monitoring timer
        self.ADS_connection_timer_ = self.create_timer(1, self.ADS_connection_timer_callback)

        # Initialize services
        self._initialize_services()

        # Active notifications storage
        self._active_notifications = {}

    def _initialize_ads_com(self, ads_com_topic, robot_station, com_config, plc_admin):
        """Initialize ADS communication based on configuration source."""
        if ads_com_topic and ads_com_topic != '':
            self._init_from_ros_service(ads_com_topic, robot_station)
        else:
            self._init_from_yaml_files(com_config, plc_admin)

    def _init_from_ros_service(self, ads_com_topic, robot_station):
        """Initialize ADS communication from ROS service."""
        self.get_logger().info(f'Using ROS service: {ads_com_topic} for configuration data')
        self.client = self.create_client(GetAdsComConfig, ads_com_topic)

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        request = GetAdsComConfig.Request()
        request.station_name = robot_station

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            com_config_data = {
                'sender_ams': response.ads_com_config.sender_ams,
                'plc_ip': response.ads_com_config.plc_ip,
                'route_name': response.ads_com_config.route_name,
                'host_name': response.ads_com_config.host_name,
                'remote_ads': response.ads_com_config.remote_ads,
            }
            plc_admin_data = {
                'plc_admin_user': response.ads_com_config.plc_admin_user,
                'plc_admin_pass': response.ads_com_config.plc_admin_password,
            }
            self.ads_com = ADSCom(com_config_data, plc_admin_data)
        else:
            self.get_logger().error('Failed to receive ADS configuration from service')
            exit(1)

    def _init_from_yaml_files(self, com_config, plc_admin):
        """Initialize ADS communication from YAML files."""
        self.get_logger().info('No "ads_com_topic" parameter found, using YAML files instead.')
        with open(com_config, 'r') as file:
            com_config_data = yaml.safe_load(file)
        with open(plc_admin, 'r') as file:
            plc_admin_data = yaml.safe_load(file)
        self.ads_com = ADSCom(com_config_data, plc_admin_data)

    def _initialize_services(self):
        """Initialize all ROS services."""
        # Basic read/write services
        self.srv_read_bool = self.create_service(ReadBool, self.get_name() + '/read_bool', self.read_bool_callback)
        self.srv_write_bool = self.create_service(WriteBool, self.get_name() + '/write_bool', self.write_bool_callback)
        self.srv_read_string = self.create_service(ReadString, self.get_name() + '/read_string', self.read_string_callback)
        self.srv_read_byte_array = self.create_service(ReadByteArray, self.get_name() + '/read_byte_array', self.read_byte_array_callback)
        self.srv_write_int = self.create_service(WriteInt, self.get_name() + '/write_int', self.write_int_callback)

        # Monitoring services
        self.srv_monitor_bool = self.create_service(MonitorBool, self.get_name() + '/monitor_bool', self.monitor_bool_callback)
        self.srv_monitor_int = self.create_service(MonitorInt, self.get_name() + '/monitor_int', self.monitor_int_callback)
        self.srv_monitor_real = self.create_service(MonitorReal, self.get_name() + '/monitor_real', self.monitor_real_callback)
        self.srv_monitor_string = self.create_service(MonitorString, self.get_name() + '/monitor_string', self.monitor_string_callback)
        self.srv_monitor_byte_array = self.create_service(MonitorByteArray, self.get_name() + '/monitor_byte_array', self.monitor_byte_array_callback)

    def ADS_connection_timer_callback(self):
        """Periodically check and maintain ADS connection."""
        if self.ads_com.connected:
            return
        try:
            self.ads_com.connect()
            self.get_logger().info("Connected to PLC")
        except Exception as e:
            self.get_logger().error(f"Connection to ADS failed: {str(e)}")

    # Basic read/write callbacks
    def read_bool_callback(self, request, response):
        try:
            response.tag_value = self.ads_com.read_by_name(request.tag_name, pyads.PLCTYPE_BOOL)
            response.success = True
            response.msg = "Successfully read bool."
        except Exception as e:
            self.get_logger().error(f"Failed to read bool: {e}")
            response.success = False
            response.msg = str(e)
        return response

    def write_bool_callback(self, request, response):
        try:
            self.ads_com.write_by_name(request.tag_name, request.tag_value, pyads.PLCTYPE_BOOL)
            response.success = True
            response.msg = f"Successfully wrote {request.tag_value} to {request.tag_name}."
        except Exception as e:
            self.get_logger().error(f"Failed to write bool: {e}")
            response.success = False
            response.msg = str(e)
        return response

    def read_string_callback(self, request, response):
        try:
            response.tag_value = bytearray(
                self.ads_com.read_by_name(request.tag_name, pyads.PLCTYPE_BYTE * (request.size + 1))).decode().strip("\00")
            response.success = True
            response.msg = "Successfully read string."
        except Exception as e:
            self.get_logger().error(f"Failed to read string: {e}")
            response.success = False
            response.msg = str(e)
        return response

    def write_int_callback(self, request, response):
        try:
            self.ads_com.write_by_name(request.tag_name, request.tag_value, pyads.PLCTYPE_INT)
            response.success = True
            response.msg = f"Successfully wrote {request.tag_value} to {request.tag_name}."
        except Exception as e:
            self.get_logger().error(f"Failed to write int: {e}")
            response.success = False
            response.msg = str(e)
        return response

    def read_byte_array_callback(self, request, response):
        try:
            response.tag_value = self.ads_com.read_by_name(request.tag_name, pyads.PLCTYPE_BYTE * request.size)
            response.success = True
            response.msg = "Successfully read byte array"
        except Exception as e:
            self.get_logger().error(f"Failed to read byte array: {e}")
            response.success = False
            response.msg = str(e)
        return response

    # Monitoring callbacks
    def monitor_bool_callback(self, request, response):
        try:
            if request.tag_name in self._active_notifications:
                response.success = True
                response.msg = f"Already monitoring {request.tag_name}"
                return response

            def callback(notification, data):
                self.get_logger().info(
                    f"BOOL {request.tag_name} changed to {notification['value']} at {notification['timestamp']}")

            handle = self.ads_com.add_device_notification(request.tag_name, pyads.PLCTYPE_BOOL, callback)
            self._active_notifications[request.tag_name] = handle
            response.success = True
            response.msg = f"Started monitoring BOOL {request.tag_name}"
        except Exception as e:
            self.get_logger().error(f"Failed to monitor BOOL: {e}")
            response.success = False
            response.msg = str(e)
        return response

    def monitor_int_callback(self, request, response):
        try:
            if request.tag_name in self._active_notifications:
                response.success = True
                response.msg = f"Already monitoring {request.tag_name}"
                return response

            def callback(notification, data):
                self.get_logger().info(
                    f"INT {request.tag_name} changed to {notification['value']} at {notification['timestamp']}")

            handle = self.ads_com.add_device_notification(request.tag_name, pyads.PLCTYPE_INT, callback)
            self._active_notifications[request.tag_name] = handle
            response.success = True
            response.msg = f"Started monitoring INT {request.tag_name}"
        except Exception as e:
            self.get_logger().error(f"Failed to monitor INT: {e}")
            response.success = False
            response.msg = str(e)
        return response

    def monitor_real_callback(self, request, response):
        try:
            if request.tag_name in self._active_notifications:
                response.success = True
                response.msg = f"Already monitoring {request.tag_name}"
                return response

            def callback(notification, data):
                self.get_logger().info(
                    f"REAL {request.tag_name} changed to {notification['value']} at {notification['timestamp']}")

            handle = self.ads_com.add_device_notification(request.tag_name, pyads.PLCTYPE_REAL, callback)
            self._active_notifications[request.tag_name] = handle
            response.success = True
            response.msg = f"Started monitoring REAL {request.tag_name}"
        except Exception as e:
            self.get_logger().error(f"Failed to monitor REAL: {e}")
            response.success = False
            response.msg = str(e)
        return response

    def monitor_string_callback(self, request, response):
        try:
            if request.tag_name in self._active_notifications:
                response.success = True
                response.msg = f"Already monitoring {request.tag_name}"
                return response

            def callback(notification, data):
                value = bytearray(notification['value']).decode().strip("\x00")
                self.get_logger().info(
                    f"STRING {request.tag_name} changed to '{value}' at {notification['timestamp']}")

            handle = self.ads_com.add_device_notification(
                request.tag_name, 
                pyads.PLCTYPE_BYTE * (request.size + 1), 
                callback)
            self._active_notifications[request.tag_name] = handle
            response.success = True
            response.msg = f"Started monitoring STRING {request.tag_name}"
        except Exception as e:
            self.get_logger().error(f"Failed to monitor STRING: {e}")
            response.success = False
            response.msg = str(e)
        return response

    def monitor_byte_array_callback(self, request, response):
        try:
            if request.tag_name in self._active_notifications:
                response.success = True
                response.msg = f"Already monitoring {request.tag_name}"
                return response

            def callback(notification, data):
                self.get_logger().info(
                    f"BYTE ARRAY {request.tag_name} changed at {notification['timestamp']}")

            handle = self.ads_com.add_device_notification(
                request.tag_name, 
                pyads.PLCTYPE_BYTE * request.size, 
                callback)
            self._active_notifications[request.tag_name] = handle
            response.success = True
            response.msg = f"Started monitoring BYTE ARRAY {request.tag_name}"
        except Exception as e:
            self.get_logger().error(f"Failed to monitor BYTE ARRAY: {e}")
            response.success = False
            response.msg = str(e)
        return response

    def destroy_node(self):
        """Clean up before shutting down."""
        for var_name, handle in self._active_notifications.items():
            try:
                self.ads_com.del_device_notification(handle)
                self.get_logger().info(f"Stopped monitoring {var_name}")
            except Exception as e:
                self.get_logger().error(f"Error removing notification for {var_name}: {e}")
        super().destroy_node()


def main():
    rclpy.init()
    node = ADSComNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()