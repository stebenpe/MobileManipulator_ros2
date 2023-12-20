import sys
import os
import json
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

pp_share = get_package_share_directory('pickplace')
pp_library =  pp_share + '/pickplace/pp_library'

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None
    
def load_json(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return json.load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None
    
# Function to convert roll, pitch and yaw to rviz readable format
def rpy_to_ypr(obj):
    return [obj[0], obj[1], obj[2], obj[5], obj[4], obj[3]]

def generate_launch_description():
    args = []
    length = len(sys.argv)
    if (len(sys.argv) >= 5):
        i = 4
        while i < len(sys.argv):
            args.append(sys.argv[i])
            i = i + 1

    #robot_description_config = load_file('omron_moma', 'MoMa_TM12.urdf')
    robot_description_config = load_file('omron_moma', 'MoMa.urdf')
    #robot_description_config = load_file('amr_visualisation', 'urdf/AMR_Platform.urdf')
    robot_description = {'robot_description' : robot_description_config}

    vis_config = get_package_share_directory('amr_visualisation') + "/param/vis_param.yaml"

    # RViz
    rviz_config_file = "src/omron_moma/config/moma_rviz.rviz"
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
    )
        
    # Joints Publisher
    joints_publisher_node = Node(
        package='amr_visualisation',
        executable='joints_publisher',
        output='screen',
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='log',
        parameters=[robot_description],
    )
    
    # Publisher linking the entire robot to "world"
    static_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='pose_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'pose', 'world']
    )
    
    # Data Points Marker
    data_points_node = Node(
        package='amr_visualisation',
        executable='data_points_marker',
        output='screen',
        parameters=[vis_config],
    )
        
    # Goals Marker
    goals_node = Node(
        package='amr_visualisation',
        executable='goals_marker',
        output='screen',
    )
    
    # Cube marker for pickplace operation
    marker_publisher_node = Node(
        package='pp_marker',
        executable='marker',
        output='screen'
    )
    
    # RViz goto point node
    goto_point_node = Node(
        package='om_aiv_navigation',
        executable='goto_point',
        output='screen',
    )
    
    # RViz goto point node
    localize_at_point_node = Node(
        package='om_aiv_navigation',
        executable='localize_at_point',
        output='screen',
    )
    
    map_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'pose']
    )
    
    laser_scans_node = Node(
        package='amr_visualisation',
        executable='laser_scans',
        output='screen',
    )
    
    # Destination Publisher
    destination_publisher_node = Node(
        package='pickplace',
        executable='destination_publisher',
        output='screen'
    )
    
    # View Transform Publisher
    view_publisher = Node(
        package='omron_moma',
        executable='view_publisher',
        output='screen'
    )
    
    tcp_publisher = Node(
        package='pickplace',
        executable='tcp_publisher',
        output='screen'
    )

    return LaunchDescription([
    	rviz_node,
        robot_state_publisher,
        joints_publisher_node,
        static_world,
        goals_node,
        data_points_node,
        goto_point_node,
        localize_at_point_node,
        map_node,
        laser_scans_node,
        marker_publisher_node,
        destination_publisher_node,
        view_publisher,
        tcp_publisher
        ])

