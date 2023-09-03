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
*  Filename:			start_world_launch.py
*  Description:         Use this file to launch e-yantra warehouse world in gazebo simulator 
*  Created:				12/07/2023
*  Last Modified:	    12/07/2023
*  Modified by:         Amit
*  Author:				e-Yantra Team
*  Copyright (c):       2023 eYantra IITB 
*  
*****************************************************************************************
'''

import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

pkg_name='eyantra_warehouse'

def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_models_dir = get_package_share_directory(pkg_name)

    install_dir = get_package_prefix(pkg_name)
    if 'GAZEBO_MODEL_PATH' in os.environ:
        gazebo_models_path = os.path.join(pkg_models_dir, 'models')
        os.environ['GAZEBO_MODEL_PATH'] = gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  install_dir + "/share"

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    # Launch gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )    

    return LaunchDescription([
        DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(pkg_models_dir, 'worlds', 'eyantra_warehouse.world'), ''],
          description='SDF world file'),
        gazebo
    ])
