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

## Description

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

To run this part of the project, ensure that you are in the '/root' folder where you have already downloaded the `rt2a1a.sh` bash file. Open a terminal and execute the following command:
```
./rt2a1a.sh
```
After executing the command, you will see three terminal windows appearing on the screen, including a Gazebo simulation with a mobile robot. Allow the system to load all the required files. Locate the terminal window titled 'user_interface', which will prompt you to press 1. Pressing 1 will initiate the movement of the mobile robot toward the randomly generated goal target. If you press 0 during goal execution, the robot will stop immediately.

### Part 2: Simulating Mobile Robot in Gazebo using ROS1/ROS2 Bridge

From the user's perspective, Part 2 of this project closely resembles Part 1. However, there are two main differences, primarily based on architecture.

The first major difference is that this part is divided into two sub-parts, with one built in ROS1 and the other in ROS2. The communication between these sub-parts is facilitated by utilizing the ROS1/ROS2 bridge. The second difference is the usage of a simple server to provide randomly generated goal coordinates to the `go_to_point` node, instead of employing an action server as done in Part 1. Consequently, when a goal cancellation request is made, the robot will not stop immediately but will finish the current goal before stopping.

In the ROS2 sub-part, we have developed the `state_machine` and `position_service` nodes as components. These components interact with the ROS1 nodes, namely `user_interface.py` and `go_to_point.py`, as well as the Gazebo simulation, facilitated by the bridge.

To run this part of the project, ensure that you are in the `/root` folder where you have already downloaded the `rt2a1b.sh` bash file. Open a terminal and execute the following command:
```
./rt2a1b.sh
```

After executing the command, you will observe four terminal windows appearing on the screen, including a Gazebo simulation featuring a mobile robot. Allow the system time to load all the necessary files. Locate the terminal window titled 'user_interface', which will prompt you to press 1. Pressing 1 will initiate the movement of the mobile robot toward the randomly generated goal target. If a goal cancellation request is made during goal execution by pressing 0, the robot will not stop immediately since an action server is not employed in this part. The robot will complete the currently assigned target before coming to a stop.

### Part 3: Simulating Mobile Robot in Vrep/CoppeliaSim

Part 3 of the project bears resemblance to Part 1, with the distinction of utilizing the Vrep/CoppeliaSim simulation environment instead of Gazebo. In this part, a communication channel is established between the Vrep scene and the `go_to_point_vrep.py` node through the development of a subscriber and a publisher.

To establish this communication channel, the `go_to_point_vrep.py` node subscribes to the `/odom` topic, where the robot's current odometry data is published by a publisher. The `go_to_point_vrep.py` node utilizes this feedback to assess the distance between the robot and the goal, subsequently computing new velocity commands based on this information. These velocity commands are then published to the `\cmd_vel` topic.

To run this part of the project, follow the steps outlined below:
1. Ensure that you are in the `/root` folder where the `ros.sh` file is downloaded. Open a terminal and execute the following command:
   ```
   source ros.sh
   ```
2. Proceed to the ROS1 workspace and launch the master by running the following command:
   ```
   roscore &
   ```
3. Navigate to the installation folder of Vrep/CoppeliaSim and launch the application by executing the following command:
   ```
   ./coppeliasim.sh
   ```
4. Wait for the Vrep simulation to start. Then, load the `pioneerROS.ttt` scene by clicking on the 'File' menu, selecting 'Open scene...', and choosing the appropriate file.
5. Once the scene is loaded, initiate the simulation by clicking the play button in the top center of the Vrep/CoppeliaSim interface. Unlike Gazebo, the simulation does not start automatically in Vrep.
6. To run the program, open another terminal window, navigate to the root folder again, and execute the following commands:
   ```
   source ros.sh
   roslaunch rt2_assignment1 sim_vrep.launch
   ```
7. After the program is loaded, it will prompt the user to press 1. Upon pressing 1, the mobile robot will commence its movement toward the randomly generated goal target. If a goal cancellation request is made by pressing 0 during goal execution, the robot will not stop immediately. It will complete the last assigned target before coming to a stop.
