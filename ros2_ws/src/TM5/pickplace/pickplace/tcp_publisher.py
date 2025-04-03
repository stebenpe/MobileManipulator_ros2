import rclpy
import rclpy.node
from tf2_ros.transform_broadcaster import TransformBroadcaster
from ament_index_python.packages import get_package_share_directory
from tm_msgs.srv import *
pp_share = get_package_share_directory('pickplace')
pp_library =  pp_share + '/pickplace/pp_library'

from pp_library import Transform

class TcpPublisher(rclpy.node.Node):
    def __init__(self):
        super().__init__('tcp_transform_node')
        self.ask_item_client = self.create_client(AskItem, "ask_item")
        while not self.ask_item_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        self.tf = Transform.TransformClass() 
        self.tfbroadcaster = TransformBroadcaster(self)
        self.future = None
        self.last_tcp_value = None  # Store the last known good TCP value
        
        # Timer for periodic updates (slower than service response time)
        self.update_timer = self.create_timer(0.1, self.update_callback)
        
        # Timer for service calls (separate from transform updates)
        self.service_timer = self.create_timer(0.5, self.service_callback)

    def service_callback(self):
        """Handle the service call separately from transform updates"""
        if self.future is not None and not self.future.done():
            self.get_logger().debug("Previous service call still in progress")
            return
            
        req = AskItem.Request()
        req.id = "tcp"
        req.item = "TCP_Value"
        req.wait_time = 1.0
        
        self.future = self.ask_item_client.call_async(req)
        self.future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        """Handle the service response"""
        try:
            response = future.result()
            if response is not None:
                self.last_tcp_value = response.value
                self.get_logger().debug(f"Received TCP value: {self.last_tcp_value}")
            else:
                self.get_logger().warn("Service call returned None")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")

    def update_callback(self):
        """Update the transform with the last known good TCP value"""
        if self.last_tcp_value is None:
            self.get_logger().debug("Waiting for first TCP value...")
            return
            
        try:
            coords_str = self.last_tcp_value[11:-1]
            coords = coords_str.split(',')
            
            # Convert coordinates
            for x in range(0, 6):
                if x <= 2:
                    coords[x] = float(coords[x]) / 1000  # mm to meters
                else:
                    coords[x] = float(coords[x]) / 57.2958  # degrees to radians
            
            # Create and send transform
            stamped_coord_params = self.tf.euler_to_stamped('flange', 'TCP', coords)
            stamped_coord_params.header.stamp = self.get_clock().now().to_msg()
            self.tfbroadcaster.sendTransform(stamped_coord_params)
            
        except Exception as e:
            self.get_logger().error(f"Error processing TCP value: {str(e)}")
            self.last_tcp_value = None  # Reset to force new service call

def main(args=None):
    rclpy.init(args=args)
    node = TcpPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()