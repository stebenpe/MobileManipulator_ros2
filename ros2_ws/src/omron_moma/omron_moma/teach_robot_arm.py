import rclpy
import json
import os
import sys
from ament_index_python.packages import get_package_share_directory
moma_share = get_package_share_directory('omron_moma')

from math import radians

from pickplace_msgs.srv import AskModbus


def start_program(startorstop):
    input(startorstop + " the robot arm learn program, then press Enter to continue...")
    
def modbus_call(node, cli, call):
    while not cli.wait_for_service(timeout_sec=1.0):
        print("Service not available...")
    req = AskModbus.Request()
    req.req = call
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    return future.result().position

def convert_rad(obj):
    x = obj[0] * 0.001
    y = obj[1] * 0.001
    z = obj[2] * 0.001
    roll = radians(obj[3])
    pitch = radians(obj[4])
    yaw = radians(obj[5])
    return [x, y, z, roll, pitch, yaw]



############################################################################3
def main():
    robot_ip = ''
    if (len(sys.argv) >= 1):
        robot_ip = sys.argv[1]
    print("Starting setup...")

    rclpy.init()
    node = rclpy.create_node("init")
    cli = node.create_client(AskModbus, 'ask_modbus')
    modbus_call(node, cli, 'init_io') #Initialise IO

    # Choose to configure for load or unload
    goal = input("Enter coordanete package name: ")

    position_name = input("Enter position name: ")

	# Set the starting position
    input("Set position, then press Enter to continue...")
    home_pos = convert_rad(modbus_call(node, cli, 'get_pos'))
    
    # Export variables to a txt file
    config = {
        position_name: home_pos
    }

    file_path = moma_share + '/' + goal + '_config.json'

    # Read existing data if the file exists
    if os.path.exists(file_path) and os.path.getsize(file_path) > 0:  # Ensure the file isn't empty
        with open(file_path, 'r') as file:
            try:
                data = json.load(file)  # Load the existing JSON array
            except json.JSONDecodeError:
                data = []  # Reset if the file is corrupted
    else:
        data = []

    # Append new data
    data.append({position_name: home_pos})

    # Write the updated list back to the file
    with open(file_path, 'w') as output:
        json.dump(data, output, indent=3)

if __name__ == '__main__':
    main()

