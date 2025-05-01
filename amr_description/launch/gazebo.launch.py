#! /usr/bin/env python3
import xacro
import os
from os.path import join
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.conditions import IfCondition
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    # with_bridge = LaunchConfiguration('with_bridge')
    # argument to specify if bridge needs to be launched
    # arg_with_bridge = DeclareLaunchArgument('with_bridge', default_value='true',
                                            # description="Set true to bridge ROS 2 & Gz topics")
    #path to xacro file
    pkg_mr_robot_desc=get_package_share_directory("amr_description")
    world_path= get_package_share_directory("amr_description")+"/worlds/world.sdf"
    
    # Include the gazebo.launch.py file
    gazebo=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('ros_gz_sim'), '/launch/gz_sim.launch.py']),launch_arguments={
                    'gz_args' : world_path + " -v 4"
                }.items())
    
    # spawn robot with rviz
    robot = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_mr_robot_desc, 'launch', 'robot.launch.py')
                ),
                # launch_arguments={
                #     'rviz': 'true',
                #     'with_bridge': 'true'
                # }.items()
            )

    
    return LaunchDescription([
        gazebo,
    ])












