#!/usr/bin/python3
# -*- coding: utf-8 -*-

''' 
*****************************************************************************************
*
*        =============================================
*           Cosmo Logistic (CL) Theme (eYRC 2023-24)
*        =============================================
*
*
*  Filename:			ebot_gazebo_launch.py
*  Description:         Use this file to spawn ebot inside e-yantra warehouse world in the gazebo simulator and publish robot states.
*  Created:				16/07/2023
*  Last Modified:	    16/07/2023
*  Modified by:         Archit
*  Author:				e-Yantra Team
*  Copyright (c):       2023 eYantra IITB 
*  
*****************************************************************************************
'''

import os
import xacro
import launch
import launch_ros
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # Getting package share directory
    pkg_share = launch_ros.substitutions.FindPackageShare(package='ebot_description').find('ebot_description')

    # Loading xacro as robot description
    xacro_file_ebot = os.path.join(pkg_share, 'models/','ebot/', 'ebot_description.xacro')
    assert os.path.exists(xacro_file_ebot), "The ebot_bot.xacro doesnt exist in "+str(xacro_file_ebot)
    robot_description_config_ebot = xacro.process_file(xacro_file_ebot)
    robot_description_ebot = robot_description_config_ebot.toxml()
    
    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ebot_description'), 'launch', 'start_world_launch.py'),
        )
    )

    # Publishing robot state
    robot_state_publisher_node_ebot = launch_ros.actions.Node(
        package='robot_state_publisher',
        name='ebot_RD',
        executable='robot_state_publisher',
        parameters=[{"robot_description": robot_description_ebot}],
        remappings=[('robot_description', 'robot_description_ebot')]
    )
    # Spawn mobile robot (ebot) at xyz location in gazebo environment
    spawn_ebot = launch_ros.actions.Node(
    	package='gazebo_ros', 
        name='ebot_spawner',
    	executable='spawn_entity.py',
        arguments=['-entity', 'ebot', '-topic', 'robot_description_ebot'],
        output='screen'
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        start_world,
        robot_state_publisher_node_ebot,
        spawn_ebot
    ])