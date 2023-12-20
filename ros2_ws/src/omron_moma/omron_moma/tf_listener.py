import rclpy
from rclpy.node import Node
import tf2_ros
from ament_index_python.packages import get_package_share_directory
pp_library =  get_package_share_directory('pickplace') + '/pickplace/pp_library'
from pickplace_msgs.srv import GetTransform


class TfListener(Node):

    def __init__(self):
        super().__init__('tf_listener')
        self.srv = self.create_service(GetTransform, 'tf_listener', self.lookup_transform)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self, spin_thread=True)
        
        rate = self.create_rate(2.0)
        

    def lookup_transform(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


def main(args=None):
    rclpy.init(args=args)

    tf_listener = TfListener()

    rclpy.spin(tf_listener)

    rclpy.shutdown()


if __name__ == '__main__':
    main()