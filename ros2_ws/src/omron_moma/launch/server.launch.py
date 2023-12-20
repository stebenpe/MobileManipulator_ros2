import sys
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

pp_share = get_package_share_directory('pickplace')
pp_library =  pp_share + '/pickplace/pp_library'


def generate_launch_description():
    args = []
    length = len(sys.argv)
    if (len(sys.argv) >= 5):
        i = 4
        while i < len(sys.argv):
            args.append(sys.argv[i])
            i = i + 1

    # TM Driver
    tm_driver_node = Node(
        package='tm_driver',
        executable='tm_driver',
        #name='tm_driver',
        output='screen',
        arguments=[str(args)[12:-2]],
    )

    modbus_server_node = Node(
        package='pickplace',
        executable='modbus_server',
        output='screen',
    )

    arcl_api = Node(
        package='om_aiv_util',
        executable='arcl_api_server',
        #name='arcl_api_server',
        output='log',
        parameters=[{
            'ip_address': "192.168.1.1",
            'port': 7171,
            'def_arcl_passwd': "omron"
        }]
    )

    ld_states = Node(
        package='om_aiv_util',
        executable='ld_states_publisher',
        #name='ld_states_publisher',
        output='screen',
        parameters=[{
            'local_ip': "192.168.1.50",
            'local_port': 7179
        }]
    )

    action_serve = Node(
        package='om_aiv_navigation',
        executable='action_server',
        #name = 'action_server',
        output='screen',
        parameters=[{
            'ip_address': "192.168.1.1",
            'port': 7171,
            'def_arcl_passwd': "omron"
        }]
    )

    return LaunchDescription([
        tm_driver_node,
        modbus_server_node,
        arcl_api,
        ld_states,
        action_serve
        ])

