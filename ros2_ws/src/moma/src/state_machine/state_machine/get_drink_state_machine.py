#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from topics_services.msg import Telemetric, Order
import random

class GetDrinkNode(Node):
    ld_status = ""
    ld_charge = 0.0
    ld_x = 0.0
    ld_y = 0.0
    ld_theta = 0.0

    def __init__(self):
        super().__init__("get_drink_state_machine")
        self.publisher_ = self.create_publisher(Telemetric, '/Telemetric', 10)
        self.subscription = self.create_subscription(Order, '/Order', self.order, 10)
        # self.subscription = self.create_subscription(Status, '/ldarcl_status', self.ld_status, 10)
        self.get_logger().info("get drink state machine has started")
        self.timer_ = self.create_timer(1, self.SendTelemetric)


    def SendTelemetric(self):
        msg = Telemetric()
        # msg.status = self.ld_status
        msg.charge = self.ld_charge
        msg.coordinates_x = self.ld_x
        msg.coordinates_y = self.ld_y
        msg.theta = self.ld_theta
        msg.power = random.randint(0,255)
        msg.eta_min = random.randint(0,10)
        msg.eta_sec = random.randint(0,60)
        msg.order_nr = random.randint(0,255)
        msg.delivery_state = random.randint(0,3)
        self.publisher_.publish(msg)


    def order(self, msg):
        drink = msg.drink
        tea_type = msg.tea_type
        deliver = msg.deliver
        strength = msg.strength
        sugar = msg.sugar

        self.get_logger().info('Received drink: {}'.format(drink))
        self.get_logger().info('Received tea_type: {}'.format(tea_type))
        self.get_logger().info('Received deliver: {}'.format(deliver))
        self.get_logger().info('Received strength: {}'.format(strength))
        self.get_logger().info('Received sugar: {}'.format(sugar))

        self.state_machine(msg)

    def state_machine(self, data):
        action_client = AmrActionClient()
        koffie_machine_result = action_client.send_goal('koffie_machine')
        if not ("Arrived at" in koffie_machine_result):
            exit()

        deliver_result = action_client.send_goal(data.deliver)
        if not ("Arrived at" in deliver_result):
            exit()

        home_result = action_client.send_goal('home')
        if not ("Arrived at" in home_result):
            exit()


    def ld_status(self, msg):
        self.ld_status = msg.status
        self.ld_charge = msg.state_of_charge
        self.ld_temperature = msg.temperature
        self.ld_x = msg.location.x
        self.ld_y = msg.location.y
        self.ld_theta = msg.location.theta


def main(args=None):
    rclpy.init(args=args)
    node = GetDrinkNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
   main()