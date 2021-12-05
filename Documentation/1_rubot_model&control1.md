# **rUBot mecanum model & control**
The objectives of this chapter are:
- Create a model of our rUBot 
- use rviz package to view the different topics and nodes
- create a model of the virtual environment
- use gazebo package to simulate the robot kinematics and dynamics
- locate the robot in our created environment
- create our firsts programs to control the robot movement with obstacle avoidance


The final model represents the real rUBot we will use in the laboratory

Interesting complementary information could be found:
- https://github.com/RBinsonB/nexus_4wd_mecanum_simulator

The rUBot mecanum robot we will work is represented in the picture:
![](./Images/1_osoyoo.png)

His main characteristics are: 
- Arduino based control for sensors & actuators
  - Servomotor actuators for the 4 mecanum wheels
  - RPlidar distance sensor
  - Raspicam 2D camera sensor
- RaspberryPi4 High-level onboard control with Ubuntu20 and ROS Noetic

## **1. rUBot mecanum model generation**

The rUBot model we will use is based on the nexus robot model developed in: https://github.com/RBinsonB/nexus_4wd_mecanum_simulator

![](./Images/1_nexus_4wd.png)

We will use this model with some modifications to take into account the different sensors installed onboard.

## 1. Nexus mecanum model generation
First of all, we have to create the "nexus_mecanum" package containing the nexus model. In case you want to create it from scratch, type:
```shell
cd ~/rubot_mecanum_ws/src
catkin_create_pkg nexus_mecanum rospy
cd ..
catkin_make
```
Then open the .bashrc file and verify the environment variables and source to the proper workspace:
```shell
source ~/rubot_mecanum_ws/devel/setup.bash
```
To create our robot model, we use URDF files (Unified Robot Description Format). URDF file is an XML format file for representing a robot model.(http://wiki.ros.org/urdf/Tutorials)

We have created 2 folders for model description:
- URDF: folder where different URDF models are located
- meshes: folder where 3D body models in stl format are located.

You can reduce the amount of code in a URDF file using Xacro package. With this package you can use constants, simple math and macros to create your robot model easier and compact.

The main parts of URDF model are:
- links: diferent bodies/plastic elements
- joints: connection between 2 links 
- sensors & actuators plugins (2D camera, LIDAR and DC motors)

The link definition contains:
- visual properties: the origin, geometry and material
- collision properties: the origin and geomnetry
- inertial properties: the origin, mass and inertia matrix

The joint definition contains:
- joint Type (fixed, continuous)
- parent and child frames
- origin frame
- rotation axis

In the case or upper left wheel:
```xml
  <link name="upper_left_wheel">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://nexus_mecanum/meshes/mecanum_wheel_left.STL" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="light_grey"/>
    </visual>
    <collision>
      <origin rpy="1.57079632679 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.0505" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.0 0.0 0.0"/>
      <mass value="0.3844"/>
      <!-- Inertia based on cylinder -->
      <inertia ixx="0.000324824" ixy="0" ixz="0" iyy="0.000480000" iyz="0" izz="0.000324824"/>
    </inertial>
  </link>
  <joint name="lower_left_wheel_joint" type="continuous">
    <origin rpy="0 0 0" xyz="0 0.042 0"/>
    <parent link="lower_left_wheel_shaft"/>
    <child link="lower_left_wheel"/>
    <axis xyz="0 1 0"/>
  </joint>
```
The rUBot model includes different sensors and actuators:

Sensors:
- a two-dimensional camera: correspondas to RBPi camera
- a 360º RPLidar A1M8 (https://www.robotshop.com/es/es/rplidar-a1m8-kit-desarrollo-escaner-laser-360-grados.html)

Actuator:
- Mecanum drive actuator: based on 4 DC motors with encoders to obtain the Odometry information

The full model contains also information about the sensor and actuator controllers using specific Gazebo plugins (http://gazebosim.org/tutorials?tut=ros_gzplugins#Tutorial:UsingGazebopluginswithROS). 

Gazebo plugins give your URDF models greater functionality and compatible with ROS messages and service calls for sensor output and motor input. 

These plugins can be referenced through a URDF file, and to insert them in the URDF file, you have to follow the sintax:
### **Camera sensor plugin**
This sensor is integrated as a link and fixed joint for visual purposes:
```xml
  <!-- 2D Camera as a mesh of actual PiCamera -->
  <link name="camera">
    <visual>
      <origin rpy="0 1.570795 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://nexus_4wd_mecanum_description/meshes/piCamera.stl" scale="0.0025 0.0025 0.0025"/>
      </geometry>
      <material name="yellow"/>
    </visual>
    <collision>
      <origin rpy="0 1.570795 0" xyz="0 0 0"/>
      <geometry>
        <box size="0.075 0.075 0.025"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 1.570795 0" xyz="0 0 0"/>
      <mass value="1e-3"/>
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
    </inertial>
  </link>
  <!-- 2D Camera JOINT base_link -->
  <joint name="joint_camera" type="fixed">
    <axis xyz="0 0 1"/>
    <origin rpy="0 0 0" xyz="0.16 0 0.05"/>
    <parent link="base_link"/>
    <child link="camera"/>
  </joint>
  ```
  A driver is needed to view the images.
```xml
  <!-- 2D Camera controller -->
  <gazebo reference="camera">
    <sensor name="camera1" type="camera">
      <update_rate>30.0</update_rate>
      <camera name="front">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>800</width>
          <height>800</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>
          <far>300</far>
        </clip>
      </camera>
      <plugin filename="libgazebo_ros_camera.so" name="camera_controller">
        <alwaysOn>true</alwaysOn>
        <updateRate>0.0</updateRate>
        <cameraName>rubot/camera1</cameraName>
        <imageTopicName>image_raw</imageTopicName>
        <cameraInfoTopicName>camera_info</cameraInfoTopicName>
        <frameName>camera</frameName>
        <hackBaseline>0.07</hackBaseline>
        <distortionK1>0.0</distortionK1>
        <distortionK2>0.0</distortionK2>
        <distortionK3>0.0</distortionK3>
        <distortionT1>0.0</distortionT1>
        <distortionT2>0.0</distortionT2>
      </plugin>
    </sensor>
  </gazebo>
  ```
### **LIDAR sensor plugin**
This sensor is integrated as a link and fixed joint for visual purposes:
```xml
  <!-- LIDAR base_scan -->
  <link name="base_scan">
    <visual name="sensor_body">
      <origin rpy="0 0 0" xyz="0 0 0.04"/>
      <geometry>
        <mesh filename="package://nexus_4wd_mecanum_description/meshes/X4.stl" scale="0.0015 0.0015 0.0015"/>
      </geometry>
      <material name="yellow"/>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0.04"/>
      <geometry>
        <cylinder length="0.01575" radius="0.0275"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0.4"/>
      <mass value="0.057"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>
  <!-- LIDAR base_scan JOINT base_link -->
  <joint name="scan_joint" type="fixed">
    <axis xyz="0 0 1"/>
    <origin rpy="0 0 0" xyz="0 0 0.09"/>
    <parent link="base_link"/>
    <child link="base_scan"/>
  </joint>
```
A driver is needed to see the 720 laser distance points:
```xml
  <!-- Laser Distance Sensor YDLIDAR X4 controller-->
  <gazebo reference="base_scan">
    <sensor name="lds_lfcd_sensor" type="ray">
      <pose>0 0 0 0 0 0</pose>
      <visualize>false</visualize>
      <update_rate>5</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>0.5</resolution>
            <min_angle>0.0</min_angle>
            <max_angle>6.28319</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.12</min>
          <max>10</max>
          <resolution>0.015</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <!-- Noise parameters based on published spec for YDLIDAR X4
              is 1.5% at half range 4m (= 60mm, "+-160mm" accuracy at max. range 8m).
              A mean of 0.0m and stddev of 0.020m will put 99.7% of samples
              within 0.16m of the true reading. -->
          <mean>0.0</mean>
          <stddev>0.02</stddev>
        </noise>
      </ray>
      <plugin filename="libgazebo_ros_laser.so" name="gazebo_ros_lds_lfcd_controller">
        <!-- topicName>/gopigo/scan</topicName -->
        <topicName>scan</topicName>
        <frameName>base_scan</frameName>
      </plugin>
    </sensor>
  </gazebo>
```
![](./Images/1_ydlidar.png)

It is important to note that:
- the number of points of real RPLidar is 720 (one each half degree)
- the number of points of simulated Lidar has to be adapted to the same 720 (by default is 360 (one each degree))
### **Mecanum drive actuator plugin**
A driver is needed to describe the kinematics.This kinematics is described in the "libgazebo_ros_planar_move.so" file and the URDF model will contain the specific gazebo plugin.

This driver is the "Planar Move Plugin" and is described in Gazebo tutorials: http://gazebosim.org/tutorials?tut=ros_gzplugins#AddingaModelPlugin

```xml
  <!-- Mecanum drive controller -->
  <gazebo>
    <plugin name="Mecanum_controller" filename="libgazebo_ros_planar_move.so">
      <commandTopic>cmd_vel</commandTopic>
      <odometryTopic>odom</odometryTopic>
      <odometryFrame>odom</odometryFrame>
      <odometryRate>50.0</odometryRate>
      <robotBaseFrame>base_footprint</robotBaseFrame>
    </plugin>
  </gazebo>
  ```
In this gazebo plugin, the kinematics of the robot configuration is defined:
- Forward kinematics: obtaining the robot POSE (odometry) with the robot wheel speeds information
- Inverse kinematics: obtaining the robot wheels speds to reach specific robot POSE (odometry)

We use a specific "display.launch" launch file where we specify the robot model we want to open in rviz with a configuration specified in "urdf.rviz":
```xml
<launch>
  <param name="robot_description" textfile="$(find nexus_mecanum)/urdf/nexus.urdf" />
  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />

  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find nexus_mecanum)/rviz/urdf_final.rviz" />
</launch>
```
Launch the ROS visualization tool to check that the model is properly built. 
RViz only represents the robot visual features. You have available all the options to check every aspect of the appearance of the model
```shell
roslaunch nexus_mecanum display.launch
```
![](./Images/1_nexus_rviz.png)

## 1.2. rUBot mecanum custom model

We can create a new model in 3D using SolidWorks and use the URDF plugin to generate the URDF file model: rubot_mecanum.urdf

This model is located in a new "rubot_mecanum" package

We add the same sensors and plugins.

We can open the new model in rviz and gazebo:

- roslaunch rubot_mecanum display.launch
- roslaunch rubot_mecanum gazebo.launch

![Getting Starter](./Images/1_rubot_mecanum2.png)

## **2. rUBot mecanum spawn in world environment**

In robotics research, always before working with a real robot, we simulate the robot behaviour in a virtual environment close to the real one. The dynamic simulation of a robot, is a better approach to examining the actual behavior of the robot rather than just using software. Rigid body mechanics, including mass and inertia, friction, damping, motor controllers, sensor detection properties, noise signals, and every aspect of the robot and the environment that can be retained in a model with reasonable accuracy is much less expensive when replicated in a simulator than if you tried to do this with physical hardware.

Gazebo is an open source 3D robotics simulator and includes an ODE physics engine and OpenGL rendering, and supports code integration for closed-loop control in robot drives. This is sensor simulation and actuator control.

We will create a new gazebo.launch file to spawn the robot in an empty world:
```shell
roslaunch nexus_mecanum gazebo.launch
roslaunch nexus_mecanum display.launch
```
```xml
<launch>
  <include file="$(find gazebo_ros)/launch/empty_world.launch" />
  <node name="spawn_model" pkg="gazebo_ros" type="spawn_model"
    args="-file $(find nexus_mecanum)/urdf/nexus.urdf -urdf -model nexus" output="screen"/>
</launch>
```
![Getting Starter](./Images/1_nexus_mecanum2.png)

>Carefull-1:
- If there is an error "libcurl: (51) SSL: no alternative certificate subject name matches target host name ‘api.ignitionfuel.org’" then follow instructions:
    - Open "~/.ignition/fuel/config.yaml" (to see the hidden files type ctrl+h)
    - replace: "api.ignitionfuel.org" with "fuel.ignitionrobotics.org"
    
- information in: https://varhowto.com/how-to-fix-libcurl-51-ssl-no-alternative-certificate-subject-name-matches-target-host-name-api-ignitionfuel-org-gazebo-ubuntu-ros-melodic/

>Carefull-2: 
- In order to kill the previous Gazebo process, type:

    killall gzserver && killall gzclient

    or type ctrl+r and kill

>Carefull-3:
- Perhaps is needed to setup your Keys again:
  ```shell
  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
  sudo apt get update
  sudo apt get upgrade
  ```
Then you can control the nexus robot with the following package:
```shell
sudo apt-get install ros-noetic-teleop-tools
sudo apt-get install ros-noetic-teleop-twist-keyboard
```
Then you will be able to control the robot with the Keyboard typing:
```shell
rosrun key_teleop key_teleop.py /key_vel:=/cmd_vel
or
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
### **2.1. Design the Project world**

Here we have first to design the project world, for exemple a maze from where our rUBot mecanum has to navigate autonomously.

There is a very useful and simple tool to design a proper world: "Building editor" in gazebo.

Open gazebo as superuser:
```shell
sudo gazebo
```
You can build your world using "Building Editor" in Edit menu
![](./Images/1_BuildingWorld1_1.png)
You can save:
- the generated model in a model folder (without extension)

Close the Builder Editor, modify the model position and add other elements if needed. Save:
- the generated world (with extension .world) in the world folder.

Once you finish is better to close the terminal you have work as superuser

#### ***Modify a created world***
- Open a terminal where you have the world you want to modify
- type: sudo gazebo ./maze1.world
- make modifications
- save your world in a Desktop directory
- close gazebo and the terminal
#### **Create world with model parts**
You can create model parts like walls of 1m or 0,5m with a geometry and color, using building editor. These parts can be saved in "home/ubuntu/building_editor_models/" and you will have acces in gazebo insert section. Then you can construct your world adding parts.

This is an exemple:
![](./Images/1_BuildingEditor.png)
### **Exercise:**
Generate a proper world corresponding to the real world we want to spawn our rUBot mecanum robot in. For exemple a maze.

Save this world as maze.world
### **2.2. Spawn the gopigo3 robot in project world**

Now, spawn the rUBot mecanum robot in our generated world. You have to create a "nexus_world.launch" file:
``` shell
roslaunch nexus_mecanum nexus_world.launch
```
![](./Images/1_nexus_mecanum3.png)

## 3. rUBot mecanum navigation control in the new world environment

Once the world has been generated we will create a ROS Package "nexus_control" to perform the autonomous navigation
```shell
cd ~/rubot_gopigo_ws/src
catkin_create_pkg nexus_control rospy std_msgs sensor_msgs geometry_msgs nav_msgs
cd ..
catkin_make
```

We will create now different navigation python files in "src" folder:
- rubot_nav1.py: to define a rubot movement with linear and angular speed
- move2_gopigo_param.py: to perform the same operation using params
- move3_gopigo_distance.py: to specify a maximum distance

Specific launch files have been created to launch the nodes and python files created above:
```shell
roslaunch gopigo3_control rubot_move1.launch
roslaunch gopigo3_control rubot_move2.launch
roslaunch gopigo3_control rubot_move3.launch
```
![Getting Started](./Images/1_rubot_move3.png)

## **gopigo3 autonomous navigation and obstacle avoidance**
In order to navigate autonomously and avoid obstacles, we have created diferent python files in "src" folder:
- rubot_lidar_test.py: to test the LIDAR distance readings and angles
- rubot_self_nav.py: to perform a simple algorithm for navigation with obstacle avoidance
- rubot_wall_follow.py: to follow a wall preciselly for mapping purposes
- rubot_go2pose.py: to reach speciffic position and orientation

we will create also a "launch" folder including the corresponding launch files
#### **1. LIDAR test**
We have created a world to test the rubot model. This world is based on a wall to verify that the LIDAR see the obstacle in the correct angle. We have to launch the "rubot_lidar_test.launch" file in the "gopigo3_control" package.

```python
#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def callback(msg):
    print len(msg.ranges)
    # values at 0 degree
    print msg.ranges[0]
    # values at 90 degree
    print msg.ranges[90]
    # values at 180 degree
    print msg.ranges[180]
    # values at 270 degree
    print msg.ranges[270]
    # values at 360 degree
    print msg.ranges[359]

rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()
```
```shell
roslaunch gopigo3_control rubot_lidar_test.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
![Getting Started](./Images/1_lidar_test.png)
#### **2. Autonomous navigation with obstacle avoidance**
We will use now the created world to test the autonomous navigation with obstacle avoidance performance. 

We have to launch the "rubot_self_nav.launch" file in the "rubot_control" package.
```shell
roslaunch gopigo3_control rubot_self_nav.launch
```
>Careful:
- we have included in launch file: gazebo spawn, rviz visualization and rubot_nav node execution 
- Verify in rviz you have to change the fixed frame to "odom" frame

![Getting Started](./Images/1_rubot_self_nav.png)
The algorithm description functionality is:
- "rubot_self_nav.py": The Python script makes the robot go forward. 
    - LIDAR is allways searching the closest distance and the angle
    - when this distance is lower than a threshold, the robot goes backward with angular speed in the oposite direction of the minimum distance angle.

#### **3. Wall Follower**
Follow the wall accuratelly is an interesting challenge to make a map with precision to apply SLAM techniques for navigation purposes.

There are 2 main tasks:
- Create a python file "rubot_wall_follower.py" to perform the wall follower in the maze of our gopigo3 robot
- Create a launch file to initialyse all the needed nodes in our system for this control task

We have developed 2 different methods for wall follower:
- Geometrical method
- Lidar ranges method

##### **a) Geometrical method**

Follow the instructions to perform the rubot_wall_follower_gm.py python program are in the notebook: 
https://github.com/Albert-Alvarez/ros-gopigo3/blob/lab-sessions/develop/ROS%20con%20GoPiGo3%20-%20S4.md
<img src="./Images/2_wall_follower1.png">
A rubot_wall_follower_gm.launch is generated to test the node within a specified world
```shell
roslaunch gopigo3_control rubot_wall_follower_gm.launch
```
<img src="./Images/1_wall_follower_gm.png">

You can see a video for the Maze wall follower process in: 
[![IMAGE_ALT](https://img.youtube.com/vi/z5sAyiFs-RU/maxresdefault.jpg)](https://youtu.be/z5sAyiFs-RU)


##### **b) Lidar ranges method**

We have created another rubot_wall_follower_rg.py file based on the reading distances from LIDAR in the ranges: front, front-right, front-left, right, left, back-right and back-left, and perform a specific actuation in function of the minimum distance readings.

Follow the instructions to create the rubot_wall_follower_rg.py python file: https://www.theconstructsim.com/wall-follower-algorithm/

<img src="./Images/1_wall_follower.png">
The algorith is based on laser ranges test and depends on the LIDAR type:
<img src="./Images/1_wall_follower2.png">

```shell
roslaunch gopigo3_control rubot_wall_follower_rg.launch
```
<img src="./Images/1_wall_ranges.png">

#### **4. Go to POSE**
Define a specific Position and Orientation as a target point to gopigo3 robot

x target point
y target point
f yaw orientation angle in deg

Modify the python script developed in turlesim control package according to the odom message type

For validation type:
```shell
roslaunch gopigo3_control rubot_go2pose.launch
```
![Getting Started](./Images/1_rubot_go2point.png)