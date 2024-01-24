#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pycomm3 import LogixDriver
from topics_services.msg import PLC_cmd, PLC_info

class PLCNode(Node):

    def __init__(self):
        super().__init__("PLC driver started")

        self.timer_ = self.create_timer(1, self.UpdatePLC)

        self.publisher_ = self.create_publisher(PLC_info, '/PLC_info', 10)
        self.subscription = self.create_subscription(PLC_cmd, '/PLC_cmd', self.PLC_cmd, 10)

        with LogixDriver('192.168.44.15/24') as self.plc:
            self.get_logger().info('PLC: {}'.format(self.plc))
            self.get_logger().info('PLC info: {}'.format(self.plc.info))

    def UpdatePLC(self):
        msg = PLC_info()
        msg.estop = self.plc.read('e-stop')
        self.publisher_.publish(msg)

    def PLC_cmd(self, msg):
        tag = msg.tag
        data = msg.data
        self.plc.write(tag, data)

def main(args=None):
    rclpy.init(args=args)
    node = PLCNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
   main()