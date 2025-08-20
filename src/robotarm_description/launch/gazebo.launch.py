from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os

def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value= os.path.join(get_package_share_directory("robotarm_description"), "urdf", "robotarm.urdf.xacro"),
        description="Absolute path to the robot urdf file"
    )

    env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", os.path.join(get_package_prefix("robotarm_description"),"share"))
    xacro_cmd = Command([
        'xacro ',
        LaunchConfiguration('model')
    ])
    robot_description = ParameterValue(xacro_cmd, value_type=str)


    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description":robot_description}]
    )

    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py"))
    )

    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py"))
    )


    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "robotarm", "-topic", "robot_description"]
    )

    return LaunchDescription([
        env_var,
        model_arg,
        robot_state_publisher,
        start_gazebo_server,
        start_gazebo_client,
        spawn_robot
    ])