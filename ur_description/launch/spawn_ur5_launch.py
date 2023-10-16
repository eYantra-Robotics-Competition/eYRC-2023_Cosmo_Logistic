'''
Author: Jaison
Credit: e-Yantra, IIT Bombay
Copyright (c) 2023 e-Yantra IITB
'''

import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path

def load_file(file_path):
    """Load the contents of a file into a string"""
    try:
        with open(file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(file_path):
    """Load a yaml file into a dictionary"""
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def run_xacro(xacro_file):
    """Run xacro and output a file in the same directory with the same name, w/o a .xacro suffix"""
    urdf_file, ext = os.path.splitext(xacro_file)
    if ext != '.xacro':
        raise RuntimeError(f'Input file to xacro must have a .xacro extension, got {xacro_file}')
    os.system(f'xacro {xacro_file} -o {urdf_file}')
    return urdf_file


def generate_launch_description():
    xacro_file = get_package_file('ur_description', 'urdf/ur5_robot.urdf.xacro')
    urdf_file = run_xacro(xacro_file)
    robot_description_arm = load_file(urdf_file)


    # TF information
    robot_state_publisher_arm = Node(
        name='robot_state_publisher',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_arm}],
        remappings=[('robot_description', 'robot_description_ur5')]
    )

    #  Visualization (parameters needed for MoveIt display plugin)
    rviz = Node(
        name='rviz',
        package='rviz2',
        executable='rviz2',
        output='screen')

    spawn_controllers_manipulator = Node(
            package="controller_manager", 
            executable="spawner",
            name="spawner_mani",
            arguments=['joint_trajectory_controller'],
            output="screen")
    

    spawn_controllers_state = Node(
            package="controller_manager", 
            executable="spawner",
            arguments=['joint_state_broadcaster'],
            output="screen")
    
    



    return LaunchDescription([
        robot_state_publisher_arm,
        spawn_controllers_manipulator,
        spawn_controllers_state,
        rviz,
        ]
    )
