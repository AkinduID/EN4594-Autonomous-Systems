#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_dir = get_package_share_directory('drone_kf')
    
    # Paths
    world_file = os.path.join(pkg_dir, 'worlds', 'drone_world.world')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'simple_drone.urdf')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', 
             '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen'
    )
    
    # Gazebo client (GUI)
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )
    
    # Spawn the drone model
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'simple_drone',
                   '-file', urdf_file,
                   '-x', '0.0',
                   '-y', '0.0', 
                   '-z', '2.0'],
        output='screen'
    )
    
    # Drone Kalman Filter node
    drone_kf_node = Node(
        package='drone_kf',
        executable='drone_kf_gazebo',
        name='drone_kf_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        gzserver,
        gzclient,
        spawn_entity,
        drone_kf_node
    ])
