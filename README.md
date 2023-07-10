# Mobile Robot Simulation in Gazebo using Action Server

## Research Track II - Assignment I
Muhammad Ali Haider Dar, _[5046263@studenti.unige.it](mailto:5046263@studenti.unige.it)_

MSc Robotics Engineering, University of Genoa, Italy

Instructor: [Prof. Carmine Recchiuto](https://rubrica.unige.it/personale/UkNDWV1r)

## Installation
Before proceeding with the instructions, please ensure that both ROS1 and ROS2 are installed on your system. This project relies on both of them. There are three branches in this repository, and each of them needs to be installed as described below.

1. To successfully deploy the code available in the "main" branch, install the code in the ROS workspace at `{ros_ws}/src` and run the following commands to build the workspace:
    * Run the following command to build the workspace:
      ```
      catkin_make
      ```
    * Navigate to the `devel/` directory:
      ```
      cd devel/
      ```
    * Source the `setup.bash` file:
      ```
      source setup.bash
      ```
2. To successfully deploy the code available in the "ros2" branch, install the code in the ROS2 workspace at `{ros2_ws}/src` and run the following commands to build the workspace:
    * Run the following command to build the workspace:
      ```
      colcon build --packages-select rt2assignment_package
      ```
    * Navigate to the `install/` directory:
      ```
      cd install/
      ```
    * Source the `local_setup.bash` file:
      ```
      source local_setup.bash
      ```
3. For the "sourcefiles" branch, which includes a set of source files required for the project, please install them in your `/root` directory. After cloning the branch, make all these files executable by running the following command for each bash file:
```
chmod +x <file_name>
```
4. To successfully proceed with this project, it is essential to have both Gazebo and Vrep simulation environments installed on your system. If you do not have them installed already, please follow the provided instructions for installation:
  **For Gazebo** 
  
  * Please follow the instructions available here: http://gazebosim.org/tutorials?tut=install_ubuntu
  
  **For Vrep/CoppeliaSim**
  
  * Download the PRO-EDU version from: http://www.coppeliarobotics.com/downloads.html
  * Vrep should be already integrated with ROS. You just need to launch the ROS master before running the V-REP (CoppeliaSim) software.
  * If there is any problem in building the plugin, you will need to recompile it by yourself: you can download it from here: CoppeliaRobotics/simExtROS (github.com).
  * Please first install xsltproc [(sudo) apt-get install xsltproc] and xmlschema [pip3 install xmlschema]
  * If specific messages/services/etc need to be supported, we have to make sure to edit files located in simExtROS/meta/, prior to recompilation. So please check if the following to message type is already included or not in the simExtROS/meta/message.txt 'geometry_msgs/Twist' 'nav_msgs/Odometry'
  * In order to build the packages, navigate to the `catkin_ws` folder and type:
    ```
    export COPPELIASIM_ROOT_DIR=~/path/to/coppeliaSim/folder
    ```
    ```
    catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```
    The packages should be generated and compiled into a library now. Copy the **devel/lib/libsimExtROS.so** file in the CoppeliaSim installation folder. The plugin is now ready to be used.
    
  *  Now to launch Vep/CoppeliaSim you can run the following command. Make sure you are in the Vep/CoppeliaSim installation folder.
     ```
     ./coppeliasim.sh 
     ```
5. After completing the previous steps, the next dependency to install is the ROS1/ROS2 bridge package. You can install this package by following the instructions provided in the following repository: https://github.com/ros2/ros1_bridge.
6. Once you have set up the ROS1/ROS2 bridge package, the next step is to modify the .bashrc file. This modification will allow you to use both the ROS1 and ROS2 frameworks simultaneously in different terminals. To do this, you need to comment out (disable) the lines related to bash files in the .bashrc file.
 ```
 #source /opt/ros/noetic/setup.bash
 #source /root/my_ros/devel/setup.bash
 #source /opt/ros/foxy/setup.bash
 #source /root/my_ros2/install/local_setup.bash
 ```
7. If you have successfully completed all the previous steps, it means that you have installed all the necessary dependencies to run the project.

The project is divided into three parts, each of which can be executed independently by following the provided instructions for each part.

### Part 1: Simulating Mobile Robot in Gazebo with Action Server

Part 1 of the project consists of four main nodes responsible for different functionalities:

1. position_service
2. state_machine_action
3. go_to_point_action.py
4. user_interface_action.py

The `user_interface_action` node facilitates communication between the user and the system. Based on the provided commands, it instructs the system accordingly. When the user presses 1 in the terminal, it sends a request to the `/user_interface` service hosted by the `state_machine_action.py` node. This informs the node that the user has requested the mobile robot to move. Upon receiving the service request from the `user_interface` node, it further requests the `/position_server` service hosted by the `position_service` node to randomly generate goal coordinates for the robot to follow.

Once the `user_interface_action` node receives the goal coordinates in response to the `/position_server` request, it passes these coordinates to the action service `/go_to_point` hosted by the `go_to_point_action.py` node. The `go_to_point_action.py` node then computes the required linear and angular velocity values for the robot to reach the specified point. Simultaneously, it publishes these values on the `cmd_vel` topic subscribed by Gazebo for robot movement.

Since `/go_to_point` is an action service, the user has the option to request goal cancellation at any point during execution. To cancel the goal, the `user_interface` node prompts the user to press 0.

To run this part of the project, ensure that you are in the '/root' folder where you have already downloaded the `rt2_assignment_1a.sh` bash file. Open a terminal and execute the following command:
```
./rt2_assignment_1a.sh
```
After executing the command, you will see three terminal windows appearing on the screen, including a Gazebo simulation with a mobile robot. Allow the system to load all the required files. Locate the terminal window titled 'user_interface', which will prompt you to press 1. Pressing 1 will initiate the movement of the mobile robot toward the randomly generated goal target. If you press 0 during goal execution, the robot will stop immediately.
