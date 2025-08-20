# RobotArm

RobotArm is a ROS 2-based project for controlling a robotic arm, featuring integration with Amazon Alexa for voice-commanded tasks and direct hardware control via an Arduino board.

## Features

- **ROS 2 Integration**: Modular packages for robot description, control, MoveIt motion planning, and remote interfaces.
- **Alexa Voice Control**: Issue high-level commands to the robot arm using Alexa. The [`alexa_interface.py`](src/robotarm_remote/robotarm_remote/alexa_interface.py) node receives intents and sends corresponding tasks to the robot via ROS 2 actions.
- **Task Server**: The [`TaskServer`](src/robotarm_remote/src/task_server.cpp) node receives tasks (e.g., pick, sleep, wake) and executes motion plans using MoveIt.
- **Serial Communication**: The [`RobotarmInterface`](src/robotarm_controller/include/robotarm_controller/robotarm_interface.hpp) hardware interface communicates with the Arduino board over serial using the [LibSerial](https://github.com/crayzeewulf/libserial) library, sending joint commands and receiving feedback.
- **Arduino Firmware**: Custom firmware ([robot_control.ino](src/robotarm_firmware/firmware/robot_control/robot_control.ino)) interprets serial commands and actuates servos for each joint.
- **Simulation Support**: Launch files for Gazebo simulation and MoveIt motion planning.
- **URDF/Xacro Description**: Detailed robot model in [URDF/Xacro](src/robotarm_description/urdf/robotarm.urdf.xacro) with support for both simulation and real hardware.

## Project Structure

- [`src/robotarm_description`](src/robotarm_description): URDF/Xacro robot model, meshes, and RViz configs.
- [`src/robotarm_controller`](src/robotarm_controller): ROS 2 hardware interface for serial communication with Arduino.
- [`src/robotarm_firmware`](src/robotarm_firmware): Arduino firmware for servo control.
- [`src/robotarm_moveit`](src/robotarm_moveit): MoveIt configuration for motion planning.
- [`src/robotarm_remote`](src/robotarm_remote): Remote interfaces, including Alexa integration and the task server.

## How It Works

1. **Voice Command**: User issues a command to Alexa (e.g., "Robot, pick up the object").
2. **Alexa Interface**: [`alexa_interface.py`](src/robotarm_remote/robotarm_remote/alexa_interface.py) translates the intent into a ROS 2 action goal and sends it to the [`TaskServer`](src/robotarm_remote/src/task_server.cpp).
3. **Task Execution**: The task server uses MoveIt to plan and execute the requested motion.
4. **Hardware Control**: The [`RobotarmInterface`](src/robotarm_controller/include/robotarm_controller/robotarm_interface.hpp) sends joint commands over serial to the Arduino.
5. **Arduino Firmware**: The firmware ([robot_control.ino](src/robotarm_firmware/firmware/robot_control/robot_control.ino)) receives commands, moves the servos, and can provide feedback if implemented.

## Getting Started

1. **Build the Workspace**:
    ```sh
    colcon build
    source install/setup.bash
    ```

2. **Launch in Simulation**:
    ```sh
    ros2 launch robotarm_bringup simulated_robot.launch.py
    ```

3. **Launch with Real Hardware**:
    ```sh
    ros2 launch robotarm_bringup real_robot.launch.py
    ```

4. **Alexa Integration**:
    - Ensure the Alexa skill is deployed and configured.
    - The [`alexa_interface.py`](src/robotarm_remote/robotarm_remote/alexa_interface.py) node must be running and configured on the web.

## Communication Protocol

- **Serial Protocol**: The robot controller sends commands like `b90,s90,e90,g0,` to the Arduino, where each letter corresponds to a joint (base, shoulder, elbow, gripper) and the number is the target angle.
- **Action Protocol**: Alexa and other clients communicate with the robot via ROS 2 actions defined in [`RobotarmTask.action`](src/robotarm_msgs/action/RobotarmTask.action).

