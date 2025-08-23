from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription , ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition

import os

def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value= os.path.join(get_package_share_directory("robotarm_description"), "urdf", "robotarm.urdf.xacro"),
        description="Absolute path to the robot urdf file"
    )

    env_var = SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", os.path.join(get_package_prefix("robotarm_description"),"share"))
    
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
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={'gz_args':'-r -s -v4 empty.sdf', 'on_exit_shutdown':'true'}.items() # Posiblemente haya que hacer un world, probaré primero sin
    )

    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={'gz_args': '-g -v4'}.items()
    )


    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "robotarm", 
                   "-topic", "robot_description",
                   '-z','0.1']
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": "True"
            },
            os.path.join(
                get_package_share_directory("robotarm_controller"),
                "config",
                "robotarm_controllers.yaml"
            )
        ],
        condition= UnlessCondition("True")
    )

    # Start arm controller
    start_arm_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'arm_controller'],
            output='screen')
    
    start_gripper_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'gripper_controller'],
            output='screen')

    # Launch joint state broadcaster
    start_joint_state_broadcaster_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'joint_state_broadcaster'],
            output='screen')

    # Launch the joint state broadcaster after spawning the robot
    load_joint_state_broadcaster_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
        target_action=spawn_robot,
        on_exit=[start_joint_state_broadcaster_cmd],))

    # Launch the arm controller after launching the joint state broadcaster
    load_arm_controller_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
        target_action=start_joint_state_broadcaster_cmd,
        on_exit=[start_arm_controller_cmd],))
    
    load_gripper_controller_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
        target_action=start_joint_state_broadcaster_cmd,
        on_exit=[start_gripper_controller_cmd],))

    return LaunchDescription([
        env_var,
        model_arg,
        robot_state_publisher,
        start_gazebo_server,
        start_gazebo_client,
        spawn_robot,
        controller_manager,
        load_arm_controller_cmd,
        load_joint_state_broadcaster_cmd,
        load_gripper_controller_cmd

    ])