import rclpy
import rclpy.node
from tf2_ros.transform_broadcaster import TransformBroadcaster
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped

class ViewTransformPublisher(rclpy.node.Node):
    marker = Marker()
    
    def __init__(self):
        super().__init__('view_transform_publisher')
        timer_period = 0.5 #seconds
        self.broadcasterpick = TransformBroadcaster(self)
        self.broadcasterplace = TransformBroadcaster(self)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.viewpicksub = self.create_subscription(TransformStamped, 'view_pick', self.set_viewpick, 10)
        self.viewplacesub = self.create_subscription(TransformStamped, 'view_place', self.set_viewplace, 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.viewpicktransform = TransformStamped()
        self.viewpicktransform.header.frame_id = "tm_base"
        self.viewpicktransform.child_frame_id = "view_pick"
        self.viewplacetransform = TransformStamped()
        self.viewplacetransform.header.frame_id = "tm_base"
        self.viewplacetransform.child_frame_id = "view_place"
        self.get_logger().info("view publisher done initializing")

    def timer_callback(self):
        self.viewpicktransform.header.stamp = self.get_clock().now().to_msg()
        self.viewplacetransform.header.stamp = self.get_clock().now().to_msg()
        self.broadcasterpick.sendTransform(self.viewpicktransform)
        self.broadcasterplace.sendTransform(self.viewplacetransform)
        
    def set_viewpick(self, msg):
        self.viewpicktransform.transform = msg.transform
        
    def set_viewplace(self, msg):
        self.viewplacetransform.transform = msg.transform
        
        
def main():
    rclpy.init()
    node = ViewTransformPublisher()
    rclpy.spin(node)
    
    
if __name__ == '__main__':
    main()