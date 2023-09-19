import os
import yaml
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch.actions import ExecuteProcess


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
  xacro_file = get_package_file('ur5_moveit', 'config/realsense2.urdf.xacro')
  urdf_file = run_xacro(xacro_file)
  
  sdf_model = LaunchConfiguration('sdf_model')
  ld = LaunchDescription([

        # DeclareLaunchArgument(name='sdf_model', 
        #                       default_value = '/home/jsn/workspace/ros_ur_driver/src/ros2_full_sensor_suite/models/full_sensor_suite/model_RS.sdf'),

        ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'], output='screen'),
        # IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'))),

        Node(name="spawn_model", 
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            # arguments=["-file", urdf_file, "-entity", "camera"])
            arguments=["-topic", '/robot_description', "-entity", "ur5", "-z", "0.1"]),

        Node(name="spawn_model_camera", 
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            # arguments=["-file", urdf_file, "-entity", "camera"])
            arguments=["-topic", '/camera/robot_description', "-entity", "camera", "-z", "0.1"])
        
  ])

  return ld