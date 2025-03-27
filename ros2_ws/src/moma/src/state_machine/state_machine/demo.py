import rclpy
import json
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from pp_library import Pickplace_Driver, Transform, TM_Exception
from ament_index_python.packages import get_package_share_directory
import time
moma_share = get_package_share_directory('omron_moma')
config_file = 'demo2'

class Coordinates:
    def __init__(self, mode):
        self.file_path = moma_share + '/' + mode + '_config.json'
        with open(self.file_path, 'r') as file:
            data_list = json.load(file)
        
        self.data = {}
        for entry in data_list:
            self.data.update(entry)

        self.home_pos = self.data.get('home', [])
        self.arm_pos1 = self.data.get('pos1', [])
        self.arm_pos2 = self.data.get('pos2', [])
        self.arm_pos3 = self.data.get('pos3', [])

        # self.data = self.load_json()
        # with open(moma_share + '/' + mode + '_config.json') as json_file:
        #     self.data = json.load(json_file)
        #     self.home_pos = self.data['home']
        #     self.arm_pos1 = self.data['pos1']
        #     self.arm_pos2 = self.data['pos2']
        #     self.arm_pos3 = self.data['pos3']

class GoalPosePublisher(Node):
    def __init__(self):
        super().__init__('goal_pose_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        #self.timer = self.create_timer(10.0, self.publish_goal_pose)  # Publish every 2 seconds
        self.counter = 0
        

    def publish_goal_pose(self):
        msg = PoseStamped()
        pickplace_driver = Pickplace_Driver.PickPlaceClass()
        coffee_machine_coords = Coordinates(config_file)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        if self.counter % 2 == 0:
            msg.pose.position.x = 2.500016212463379
            msg.pose.position.y = -2.8973746299743652
            pickplace_driver.set_position(coffee_machine_coords.home_pos)
        else:
            pickplace_driver.set_position(coffee_machine_coords.view_pick)
            msg.pose.position.x = 1.3960500955581665
            msg.pose.position.y = -2.8885107040405273
        
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published goal pose: {msg.pose.position.x}, {msg.pose.position.y}')
        
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = GoalPosePublisher()
    node.get_logger().info('hello')
    msg = PoseStamped()
    pickplace_driver = Pickplace_Driver.PickPlaceClass()
    coffee_machine_coords = Coordinates(config_file)
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = 'world'

    msg.pose.position.x = 2.500016212463379
    msg.pose.position.y = -2.8973746299743652
        
    msg.pose.position.z = 0.0
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = 0.0
    msg.pose.orientation.w = 1.0
        
    #pickplace_driver.set_position(coffee_machine_coords.view_pick)
    pickplace_driver.open()
    pickplace_driver.set_position(coffee_machine_coords.home_pos)
    pickplace_driver.close()
    msg.header.stamp = node.get_clock().now().to_msg()
    node.publisher_.publish(msg)
    node.get_logger().info(f'Published goal pose: {msg.pose.position.x}, {msg.pose.position.y}')
    
    
    msg.pose.position.x = 1.3960500955581665
    msg.pose.position.y = -2.8885107040405273

    time.sleep(10)

    pickplace_driver.set_position(coffee_machine_coords.arm_pos1)
    pickplace_driver.set_position(coffee_machine_coords.arm_pos2)
    pickplace_driver.set_position(coffee_machine_coords.arm_pos3)
    pickplace_driver.set_position(coffee_machine_coords.arm_pos2)
    pickplace_driver.open()
    pickplace_driver.set_position(coffee_machine_coords.home_pos)
    pickplace_driver.close()

    msg.header.stamp = node.get_clock().now().to_msg()
    node.publisher_.publish(msg)
    node.get_logger().info(f'Published goal pose: {msg.pose.position.x}, {msg.pose.position.y}')
    
    pickplace_driver.set_position(coffee_machine_coords.arm_pos1)
    pickplace_driver.open()
    pickplace_driver.set_position(coffee_machine_coords.home_pos)
    pickplace_driver.close()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
