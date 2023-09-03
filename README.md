# How to use this branch-

## Some changes before you build the package
* Changes in SkyX (Gazebo)

  *  `SkyX_Moon.fragment` file located at /usr/share/gazebo-11/media/skyx/ needs to be updated with the one added here- https://github.com/eYantra-Robotics-Competition/eyrc-23-24-cl/tree/task_0/eyantra_warehouse/config/SkyX_Moon.fragment.

  *  `SkyX_Moon.png` file located at /usr/share/gazebo-11/media/skyx/ needs to be updated with the one added here- https://github.com/eYantra-Robotics-Competition/eyrc-23-24-cl/tree/task_0/eyantra_warehouse/config/SkyX_Moon.png.

* Install these packages if you haven't installed before:

  * Install pip3 using `sudo apt install python3-pip`
  
  * Install transforms3d `sudo pip3 install transforms3d`

  * Install gazebo-ros using `sudo apt install ros-humble-gazebo-ros`
  
  * Install gazebo-plugins using `sudo apt install ros-humble-gazebo-plugins`

  * Install xacro using `sudo apt install ros-humble-xacro`

  * Install tf_transformations using `sudo apt install ros-humble-tf-transformations`

## To spawn the robot in the Gazebo Simulator, use-
`ros2 launch ebot_description ebot_gazebo_launch.py`

## To control the robot using keyboard, use-
`ros2 run teleop_twist_keyboard teleop_twist_keyboard`
