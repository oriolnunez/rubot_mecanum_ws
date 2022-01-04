# **Migration to ROS2**

ROS2 structure is based on:

ROS2 is a very good choice.

The main advantages are:
- 

Some interesting projects:
- https://github.com/noshluk2/ROS2-Self-Driving-Car-AI-using-OpenCV

## 1. **ROS2 Installation**
Follow the instructions on:
https://docs.ros.org/en/foxy/Installation.html

Some other installations needed:
- Colcon compilation utility (https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html)
```shell
sudo apt install python3-colcon-common-extensions
```
- Visual Studio Code with some extensions:
    - python 
    - python for VS Code
    - python Intellisense
    - XML (Red-Hat)
    - XML Tools
    - Code runner
    - ROS


## 2. **Create workspace**
Create "self_driving_car_ws2" workspace with "src" folder

Compile the ws:
```shell
colcon build
```

Create your first Package: https://docs.ros.org/en/foxy/Tutorials/Creating-Your-First-ROS2-Package.html
