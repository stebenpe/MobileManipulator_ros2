import rclpy
import json
from rclpy.node import Node
from om_aiv_navigation.goto_goal import AmrActionClient
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


class GoalPosePublisher(Node):
    def __init__(self):
        super().__init__('goal_pose_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)


def main(args=None):
    rclpy.init(args=args)
    node = GoalPosePublisher()
    action_client = AmrActionClient()
    pickplace_driver = Pickplace_Driver.PickPlaceClass()
    coffee_machine_coords = Coordinates(config_file)
    
    for x in range(6):
        Goal1_result = action_client.send_goal('Goal2')
        if not ("Arrived at" in Goal1_result):
            exit()
        # pickplace_driver.set_position(coffee_machine_coords.home_pos)
        node.get_logger().info('set pos2')
        pickplace_driver.set_position(coffee_machine_coords.arm_pos2)
        node.get_logger().info('set pos3')
        # pickplace_driver.set_position(coffee_machine_coords.arm_pos3)
        # pickplace_driver.set_position(coffee_machine_coords.arm_pos2)
        # pickplace_driver.open()
        # pickplace_driver.set_position(coffee_machine_coords.home_pos)
        # pickplace_driver.close()

        # Goal1_result = action_client.send_goal('Goal2')
        # if not ("Arrived at" in Goal1_result):
        #     exit()

        # pickplace_driver.set_position(coffee_machine_coords.arm_pos2)
        # pickplace_driver.set_position(coffee_machine_coords.arm_pos3)
        # pickplace_driver.set_position(coffee_machine_coords.arm_pos2)
        # pickplace_driver.open()
        # pickplace_driver.set_position(coffee_machine_coords.home_pos)
        # pickplace_driver.close()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
