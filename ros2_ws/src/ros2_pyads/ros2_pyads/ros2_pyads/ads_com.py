import pyads


class ADSCom:

    def __init__(self, com_config, plc_admin):
        """
        Initialize the ADS communication object.

        :param com_config: Object containing the ADS communication configuration YAML data
        :param plc_admin: Object containing the PLC admin credentials YAML data
        """
        self.com_config = com_config
        self.plc_admin = plc_admin

        # Get the parameters from the config file
        self.sender_ams = self.com_config['sender_ams']
        self.plc_ip = self.com_config['plc_ip']
        self.route_name = self.com_config['route_name']
        self.host_name = self.com_config['host_name']
        self.remote_ads = self.com_config['remote_ads']

        # Get Admin credentials
        self.plc_admin_user = self.plc_admin['plc_admin_user']
        self.plc_admin_pass = self.plc_admin['plc_admin_pass']
        self.connected = False
        self.plc = None

    def connect(self):
        # Initialize the route
        self.initialize_route()
        self.plc = pyads.Connection(self.remote_ads, pyads.PORT_TC3PLC1, self.plc_ip)
        self.plc.open()
        self.connected = True

    def initialize_route(self):
        """
        Initialize the route to the PLC using the sender AMS ID, PLC IP, route name, host name, and PLC admin
        credentials.
        """
        pyads.open_port()
        pyads.set_local_address(self.sender_ams)
        pyads.add_route_to_plc(
            sending_net_id=self.sender_ams,
            adding_host_name=self.host_name,
            ip_address=self.plc_ip,
            username=self.plc_admin_user,
            password=self.plc_admin_pass,
            route_name=self.route_name)
        pyads.close_port()

    def read_by_name(self, var_name, var_type):
        """
        Read a variable from the PLC by name.

        :param var_name: The name of the variable to read (e.g., 'MAIN.testVar')
        :param var_type: The type of the variable to read (e.g., pyads.PLCTYPE_DWORD)

        :return: The value of the variable
        """
        if not hasattr(self, 'plc') or not self.connected:
            self.connect()

        try:
            var = self.plc.read_by_name(var_name, var_type)
            return var
        except Exception as e:
            self.connected = False
            raise e

    def write_by_name(self, var_name, var_value, var_type):
        """
            Write a variable to the PLC by name.
    
            Returns:
            bool: True if write succeeded, False if failed
        """
        try:
            self.plc.write_by_name(var_name, var_value, var_type)
            return True  # Success
        except Exception as e:
            # self.get_logger().error(f'Failed to write {var_name}: {str(e)}')
            self.connected = False
            return False  # Failure

    def add_device_notification(self, var_name, var_type, callback):
        """Add a device notification callback."""
        if not hasattr(self, 'plc') or not self.connected:
            self.connect()
        try:
            attrib = pyads.NotificationAttrib(
                length=pyads.size_of_structure(var_type))
            return self.plc.add_device_notification(var_name, callback, attrib, var_type)
        except Exception as e:
            self.connected = False
            raise e

    def del_device_notification(self, handle):
        """Remove a device notification."""
        if hasattr(self, 'plc') and self.connected:
            try:
                self.plc.del_device_notification(handle)
            except Exception as e:
                self.connected = False
                raise e
