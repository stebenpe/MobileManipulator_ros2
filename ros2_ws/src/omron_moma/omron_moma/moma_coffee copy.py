import rclpy
import time
import math
import json
import tf2_ros
import functools
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
from topics_services.msg import Telemetric, Order

home_goal = 'home'
coffee_machine_goal = 'coffee_machine'

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
        return tf.get_picks(new_vbase, vbase_name, True)
    elif (vbase_name == "vbase_pick_1"):
        return tf.get_picks(new_vbase, vbase_name, True)
    elif (vbase_name == "vbase_pick_2"):
        return tf.get_picks(new_vbase, vbase_name, True)
    elif (vbase_name == "vbase_pick_3"):
        return tf.get_picks(new_vbase, vbase_name, True)
    elif (vbase_name == "vbase_place"):
        return tf.get_places(new_vbase, vbase_name, True)
    elif (vbase_name == "vbase_place_1"):
        return tf.get_places(new_vbase, vbase_name, True)
    elif (vbase_name == "vbase_place_2"):
        return tf.get_places(new_vbase, vbase_name, True)
    elif (vbase_name == "vbase_place_3"):
        return tf.get_places(new_vbase, vbase_name, True)
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
    ready = client.wait_for_service(timeout_sec=15.0)
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
            if (mode == 'home'):
                self.vbase_pick = self.data['vbase_pick']
                self.vjob_name_place = self.data['vjob_name_place']
                self.vbase_place_1 = self.data['vbase_place_1']
                self.vbase_place_2 = self.data['vbase_place_2']
                self.vbase_place_3 = self.data['vbase_place_3']
            elif (mode == 'coffee_machine'):
                self.vjob_name_place = self.data['vjob_name_place']
                self.vbase_pick_1 = self.data['vbase_pick_1']
                self.vbase_pick_2 = self.data['vbase_pick_2']
                self.vbase_pick_3 = self.data['vbase_pick_3']
                self.vbase_place = self.data['vbase_place']
            else:
                print('not supported mode')

            self.home_pos = self.data['home_pos']
            self.vjob_name = self.data['vjob_name']
            self.view_pick = self.data['view_pick']
            self.view_place = self.data['view_place']

class TMHandler:
    def __init__(self, node, pickplace_driver):
        self.node = node
        self.pickplace_driver = pickplace_driver
        self.tf = Transform.TransformClass()
        self.cli = node.create_client(AskModbus, 'ask_modbus')
        self.flagpublisher = self.node.create_publisher(MoveCube, 'objectflag', 10)

        self.pickplace_driver.wait_tm_connect()
      
    # Executes the pickplace sequence at the designated goal
    def execute_tm(self, coord, mug, coffee_machine_pick):
        if (mug == 0): 
            self.tf.add_vbases(coord.vbase_pick, coord.vbase_place, 0, False)
            print("mug = 0")
        elif (mug == 1 and coffee_machine_pick == False):
            self.tf.add_vbases(coord.vbase_pick_1, coord.vbase_place, 1, False)
            print("mug = 1")
        elif (mug == 2 and coffee_machine_pick == False):
            self.tf.add_vbases(coord.vbase_pick_2, coord.vbase_place, 2, False)
            print("mug = 2")
        elif (mug == 3 and coffee_machine_pick == False):
            self.tf.add_vbases(coord.vbase_pick_3, coord.vbase_place, 3, False)
            print("mug = 3")
        elif (mug == 1 and coffee_machine_pick == True):
            self.tf.add_vbases(coord.vbase_pick, coord.vbase_place_1, 1, False)
            print("mug = 1")
        elif (mug == 2 and coffee_machine_pick == True):
            self.tf.add_vbases(coord.vbase_pick, coord.vbase_place_2, 2, False)
            print("mug = 2")
        elif (mug == 3 and coffee_machine_pick == True):
            self.tf.add_vbases(coord.vbase_pick, coord.vbase_place_3, 3, False)
            print("mug = 3")
        else:
            self.node.get_logger().error("mug not defined")
            exit()

        self.pickplace_driver.set_position(coord.view_pick)
        if not self.pickplace_driver.error:
            # get positions of the pick operation
            if (mug == 0):
                print("mug = 0")
                pick, safepick = get_positions(self.pickplace_driver, self.node, self.cli, self.tf, "vbase_pick", coord.vjob_name)
            elif (mug == 1 and coffee_machine_pick == False):
                print("pick cup from holder 1")
                pick, safepick = get_positions(self.pickplace_driver, self.node, self.cli, self.tf, "vbase_pick_1", coord.vjob_name)
            elif (mug == 2 and coffee_machine_pick == False):
                print("pick cup from holder 2")
                pick, safepick = get_positions(self.pickplace_driver, self.node, self.cli, self.tf, "vbase_pick_2", coord.vjob_name)
            elif (mug == 3 and coffee_machine_pick == False):
                print("pick cup from holder 3")
                pick, safepick = get_positions(self.pickplace_driver, self.node, self.cli, self.tf, "vbase_pick_3", coord.vjob_name)
            elif (coffee_machine_pick == True):
                print("pick cup from coffee machine")
                pick, safepick = get_positions(self.pickplace_driver, self.node, self.cli, self.tf, "vbase_pick", coord.vjob_name)
            else:
                print("mug = nan")
                self.node.get_logger().error("mug not defined")
                exit()

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
            if (mug == 0):
                print("mug = 0")
                place, safeplace = get_positions(self.pickplace_driver, self.node, self.cli, self.tf, "vbase_place", coord.vjob_name_place)
            elif (mug ==1 and coffee_machine_pick == True):
                print("place cup in holder 1")
                place, safeplace = get_positions(self.pickplace_driver, self.node, self.cli, self.tf, "vbase_place_1", coord.vjob_name_place)
            elif (mug ==2 and coffee_machine_pick == True):
                print("place cup in holder 2")
                place, safeplace = get_positions(self.pickplace_driver, self.node, self.cli, self.tf, "vbase_place_2", coord.vjob_name_place)
            elif (mug ==3 and coffee_machine_pick == True):
                print("place cup in holder 1")
                place, safeplace = get_positions(self.pickplace_driver, self.node, self.cli, self.tf, "vbase_place_3", coord.vjob_name_place)
            elif (coffee_machine_pick == False):
                print("place cup in coffee machine")
                place, safeplace = get_positions(self.pickplace_driver, self.node, self.cli, self.tf, "vbase_place", coord.vjob_name_place)
            else:
                print("mug = nan")
                self.node.get_logger().error("mug not defined")
                exit()

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

def get_coffee(node):
    try:     
        #TODO activate LD90

        node.delivery_state = 1

        #go to coffee machine
        # coffee_machine_result = node.action_client.send_goal(coffee_machine_goal)
        # if not ("Arrived at" in coffee_machine_result):
        #     node.get_logger().info("Failed to arrive at the coffee machine!")
        #     exit()

        #TODO activate TM5

        node.delivery_state = 2

        #place cup in coffee machine and select drink
        publish_view(node.Goal2_coords, node.viewpickpub, node.viewplacepub)
        print("Picking the coffee")
        node.tm_handler.execute_tm(node.Goal2_coords, node.mug)
        print("Picked the coffee")
        
        #wait for the coffee pouring
        time.sleep(15)

        #pick the coffee from the coffee machine
        publish_view(node.Goal1_coords, node.viewpickpub, node.viewplacepub)
        node.tm_handler.execute_tm(node.Goal1_coords, node.mug)

        # Set the TM to move to the designated home position
        current_position = get_current_pos(node)
        if not check_same_positions(current_position, node.Goal1_coords.home_pos):
            node.pickplace_driver.set_position(node.Goal1_coords.home_pos)

        #TODO activate LD90

        #go to deliver place
        deliver_result = node.action_client.send_goal(node.deliver)
        if not ("Arrived at" in deliver_result):
            node.get_logger().info("Failed to arrive at the coffee machine!")
            exit()

        node.delivery_state = 3

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

def order(msg, node):
    node.drink = msg.drink
    node.tea_type = msg.tea_type
    node.deliver_location = msg.deliver
    node.strength = msg.strength
    node.sugar = msg.sugar

    node.get_logger().info('Received drink: {}'.format(node.drink))
    node.get_logger().info('Received tea_type: {}'.format(node.tea_type))
    node.get_logger().info('Received deliver: {}'.format(node.deliver_location))
    node.get_logger().info('Received strength: {}'.format(node.strength))
    node.get_logger().info('Received sugar: {}'.format(node.sugar))

    node.should_spin = False
    # get_coffee(node)

# def SendTelemetric(node):
#     msg = Telemetric()
#     # msg.status = self.ld_status
#     msg.charge = node.ld_charge
#     msg.coordinates_x = node.ld_x
#     msg.coordinates_y = node.ld_y
#     msg.theta = node.ld_theta
#     # msg.power = random.randint(0,255)
#     # msg.eta_min = random.randint(0,10)
#     # msg.eta_sec = random.randint(0,60)
#     # msg.order_nr = random.randint(0,255)
#     msg.delivery_state = node.delivery_state
#     node.publisher_.publish(msg)

def main():
    rclpy.init()
    node = rclpy.create_node('moma_coffee_node')

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
    Goal1_coords = Coordinates(home_goal)
    Goal2_coords = Coordinates(coffee_machine_goal)

    mug = 1

    #topics for webserver
    node.publisher_ = node.create_publisher(Telemetric, '/Telemetric', 10)
    order_with_node = functools.partial(order, node=node)
    node.subscription = node.create_subscription(Order, '/Order', order_with_node, 10)

    #callback for telemetric update
    # node.timer_ = node.create_timer(1, SendTelemetric(node))

    #TODO safety plc set robotarm active

    # Set the TM to move to the designated home position
    current_position = get_current_pos(node, cli)
    if not check_same_positions(current_position, Goal2_coords.home_pos):
        pickplace_driver.set_position(Goal2_coords.home_pos)

    #TODO safety plc set AMR active

    # home_result = node.action_client.send_goal(home_goal)
    # if not ("Arrived at" in home_result):
    #     node.get_logger().info("Failed to arrive at home station!")
    #     exit()
    
    node.deliver_location = ""
    ld_charge = 0
    ld_x = 0
    ld_y = 0
    ld_theta = 0
    power = 0
    eta_min = 0
    eta_sec = 0
    order_nr = 0
    delivery_state = 0

    node.should_spin = True

    # Start spinning
    while node.should_spin and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=1)  # Adjust timeout as needed


    publish_view(Goal2_coords, viewpickpub, viewplacepub)
    tm_handler.execute_tm(Goal2_coords, 1, False)
    delivery_state = 1

    # Set the TM to move to the designated home position
    current_position = get_current_pos(node, cli)
    if not check_same_positions(current_position, Goal2_coords.home_pos):
        pickplace_driver.set_position(Goal2_coords.home_pos)

    #wait for the coffee pouring
    time.sleep(15)

    #pick the coffee from the coffee machine
    publish_view(Goal1_coords, viewpickpub, viewplacepub)
    tm_handler.execute_tm(Goal1_coords, 1, True)

    # Set the TM to move to the designated home position
    current_position = get_current_pos(node)
    if not check_same_positions(current_position, Goal1_coords.home_pos):
        pickplace_driver.set_position(Goal1_coords.home_pos)

    #TODO activate LD90

    #go to deliver place
    delivery_state = 2
    deliver_result = action_client.send_goal(node.deliver_location)
    if not ("Arrived at" in deliver_result):
        node.get_logger().info("Failed to arrive at the coffee machine!")
        exit()

    delivery_state = 3

    zero = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    call_set_parameters(node, zero)
    flagpublisher = node.create_publisher(MoveCube, 'objectflag', 10)
    msg = MoveCube()
    msg.parent = "world"
    msg.coordinates = zero
    flagpublisher.publish(msg)

    rclpy.spin(node)
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()