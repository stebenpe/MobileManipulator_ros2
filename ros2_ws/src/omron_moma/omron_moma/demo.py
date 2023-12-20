import rclpy
import time
import math
import json
import tf2_ros
from ament_index_python.packages import get_package_share_directory
moma_share = get_package_share_directory('omron_moma')
pp_library =  get_package_share_directory('pickplace') + '/pickplace/pp_library'

from pp_library import Pickplace_Driver, Transform, TM_Exception
from om_aiv_navigation.goto_goal import AmrActionClient
from pickplace_msgs.srv import AskModbus
from pickplace_msgs.msg import MoveCube
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

start_goal = 'Goal2'
end_goal = 'Goal1'

# Get the coordinates of the new vision base
def get_base(node, cli):
    while not cli.wait_for_service(timeout_sec=1.0):
        print("Service not available...")
    req = AskModbus.Request()
    req.req = 'get_base'
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    return future.result().position

def get_current_pos(node, cli):
    while not cli.wait_for_service(timeout_sec=1.0):
        print("Service not available...")
    req = AskModbus.Request()
    req.req = 'get_pos'
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    return future.result().position

# Get the new pick or place positions w.r.t the new vision base
def get_positions(listener, node, cli, tf, vbase_name, vjob_name):
    listener.exit_script()
    listener.change_base(vjob_name)
    time.sleep(0.1)
    new_vbase = get_base(node, cli)
    time.sleep(0.1)
    listener.change_base("RobotBase")
    time.sleep(0.1)
    if (vbase_name == "vbase_pick"):
        return tf.get_picks(new_vbase, vbase_name)
    elif (vbase_name == "vbase_place"):
        return tf.get_places(new_vbase, vbase_name)
    else:
        return new_vbase

"""
    -INPUT IS IN mm AND deg, SO IT IS CONVERTED TO m AND rad HERE
Converts euler roll, pitch, yaw to quaternion
quat = [w, x, y, z]
"""
def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr #w
    q[1] = cy * cp * sr - sy * sp * cr #x
    q[2] = sy * cp * sr + cy * sp * cr #y
    q[3] = sy * cp * cr - cy * sp * sr #z

    return q

# Set the destination transform location
def call_set_parameters(node, coordinates):
    # create client
    client = node.create_client(
        SetParameters,
        'destination_node/set_parameters'.format_map(locals()))

    # call as soon as ready
    ready = client.wait_for_service(timeout_sec=5.0)
    if not ready:
        raise RuntimeError('Wait for service timed out')

    request = SetParameters.Request()
    param_values = ParameterValue(type = ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value = coordinates)
    request.parameters = [Parameter(name = 'destination_param', value = param_values)]
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future) 

    # handle response
    response = future.result()
    if response is None:
        e = future.exception()
        raise RuntimeError(
            'Exception while calling service of node '
            "'{args.node_name}': {e}".format_map(locals()))
    return response


def publish_view(Goal_coords, viewpick_pub, viewplace_pub):
    transform = TransformStamped()
    transform.transform.translation.x = Goal_coords.view_pick[0]
    transform.transform.translation.y = Goal_coords.view_pick[1]
    transform.transform.translation.z = Goal_coords.view_pick[2]
    temp = quaternion_from_euler(Goal_coords.view_pick[3], Goal_coords.view_pick[4], Goal_coords.view_pick[5])
    transform.transform.rotation.w = temp[0]
    transform.transform.rotation.x = temp[1]
    transform.transform.rotation.y = temp[2]
    transform.transform.rotation.z = temp[3]
    viewpick_pub.publish(transform)
    transform.transform.translation.x = Goal_coords.view_place[0]
    transform.transform.translation.y = Goal_coords.view_place[1]
    transform.transform.translation.z = Goal_coords.view_place[2]
    temp = quaternion_from_euler(Goal_coords.view_place[3], Goal_coords.view_place[4], Goal_coords.view_place[5])
    transform.transform.rotation.w = temp[0]
    transform.transform.rotation.x = temp[1]
    transform.transform.rotation.y = temp[2]
    transform.transform.rotation.z = temp[3]
    viewplace_pub.publish(transform)
    

# Creates a class for coordinates from the teach_setup config.txt to be initialised
# The paramater 'mode' will be either 'load' or 'unload'
class Coordinates:
    def __init__(self, mode):
        with open(moma_share + '/' + mode + '_config.txt') as json_file:
            self.data = json.load(json_file)
            self.home_pos = self.data['home_pos']
            self.vjob_name = self.data['vjob_name']
            self.view_pick = self.data['view_pick']
            self.view_place = self.data['view_place']
            self.vbase_pick = self.data['vbase_pick']
            self.vbase_place = self.data['vbase_place']

class TMHandler:
    def __init__(self, node, pickplace_driver):
        self.node = node
        self.pickplace_driver = pickplace_driver
        self.tf = Transform.TransformClass()
        self.cli = node.create_client(AskModbus, 'ask_modbus')
        self.flagpublisher = self.node.create_publisher(MoveCube, 'objectflag', 10)

        self.pickplace_driver.wait_tm_connect()
      
    # Executes the pickplace sequence at the designated goal
    def execute_tm(self, coord):
        self.tf.add_vbases(coord.vbase_pick, coord.vbase_place)
     
        self.pickplace_driver.set_position(coord.view_pick)
        if not self.pickplace_driver.error:
            # get positions of the pick operation
            pick, safepick = get_positions(self.pickplace_driver, self.node, self.cli, self.tf, "vbase_pick", coord.vjob_name)
            # set the parameters of the flange destination
            call_set_parameters(self.node, pick)
            # sets the cube display to appear at the pick location
            msg = MoveCube()
            msg.parent = "tm_base"
            msg.coordinates = pick
            self.flagpublisher.publish(msg)
            # moves the grippers above the object and open the grippers
            self.pickplace_driver.set_position(safepick)
            self.pickplace_driver.open()
            # moves the grippers down to the object picking location and close the grippers
            self.pickplace_driver.set_position(pick)
            self.pickplace_driver.close()
            # sets marker to be joined to the end effector
            msg.parent = "EOAT"
            msg.coordinates = pick
            self.flagpublisher.publish(msg)
            # moves the gripper back to above the object
            self.pickplace_driver.set_position(safepick)

        self.pickplace_driver.set_position(coord.view_place)
        if not self.pickplace_driver.error:
            # get positions of the place operation
            place, safeplace = get_positions(self.pickplace_driver, self.node, self.cli, self.tf, "vbase_place", coord.vjob_name)
            # set the parameters of the flange destination
            call_set_parameters(self.node, place)
            # moves the gripper to above the place location
            self.pickplace_driver.set_position(safeplace)
            # moves the gripper to the place location and open the grippers
            self.pickplace_driver.set_position(place)
            self.pickplace_driver.open()
            # sets the cube display to appear at the place location
            msg = MoveCube()
            msg.parent = "tm_base"
            msg.coordinates = place
            self.flagpublisher.publish(msg)
            # moves the gripper back to above the object
            self.pickplace_driver.set_position(safeplace)

        self.pickplace_driver.set_position(coord.home_pos)

        if self.pickplace_driver.error:
            self.node.get_logger().info("TM ERROR, SHUTTING DOWN PROGRAM!")
            exit()

"""
Check if the tm is moving to the same location
"""
def check_same_positions(current, goal):
    if (round(current[0]/1000, 3) == round(goal[0],3) and
        round(current[1]/1000, 3) == round(goal[1],3) and
        round(current[2]/1000, 3) == round(goal[2],3) and
        round(current[3]/57.2958, 3) == round(goal[3],3) and
        round(current[4]/57.2958, 3) == round(goal[4],3) and
        round(current[5]/57.2958, 3) == round(goal[5],3)):
            return True
    return False

def main():
    rclpy.init()
    node = rclpy.create_node('demo_node')

    # Initialise gripper using modbus
    cli = node.create_client(AskModbus, 'ask_modbus')
    while not cli.wait_for_service(timeout_sec=1.0):
        print("Service not available...")
    req = AskModbus.Request()
    req.req = 'init_io'
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    pickplace_driver = Pickplace_Driver.PickPlaceClass()
    tm_handler = TMHandler(node, pickplace_driver)
    action_client = AmrActionClient()
    viewpickpub = node.create_publisher(TransformStamped, 'view_pick', 10)
    viewplacepub = node.create_publisher(TransformStamped, 'view_place', 10)
    # Load the coordinates taught in teach_setup for the respective goals
    Goal1_coords = Coordinates(end_goal)
    Goal2_coords = Coordinates(start_goal)

    # Set the TM to move to the designated home position
    current_position = get_current_pos(node, cli)
    if not check_same_positions(current_position, Goal1_coords.home_pos):
        # node.get_logger().info(str(current_position))
        # node.get_logger().info(str(Goal1_coords.home_pos))
        pickplace_driver.set_position(Goal1_coords.home_pos)
    
    try:     
        goal2result = action_client.send_goal(start_goal)
        if not ("Arrived at" in goal2result):
            node.get_logger().info("Failed to arrive at goal!")
            exit()
        publish_view(Goal2_coords, viewpickpub, viewplacepub)
        tm_handler.execute_tm(Goal2_coords)
        
        goal1result = action_client.send_goal(end_goal)
        if not ("Arrived at" in goal1result):
            node.get_logger().info("Failed to arrive at goal!")
            exit()
        publish_view(Goal1_coords, viewpickpub, viewplacepub)
        tm_handler.execute_tm(Goal1_coords)
        zero = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        call_set_parameters(node, zero)
        flagpublisher = node.create_publisher(MoveCube, 'objectflag', 10)
        msg = MoveCube()
        msg.parent = "world"
        msg.coordinates = zero
        flagpublisher.publish(msg)

    except KeyboardInterrupt:
        node.get_logger().info("Program shut down!")
    except TM_Exception.TM_Exception as e:
        node.get_logger().error(str(e))

    
if __name__ == '__main__':
    main()

