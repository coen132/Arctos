# Arctos
This is a ROS2 package for the Arctos arm. make sure [ROS2](https://docs.ros.org/en/humble/Installation.html) and [MoveIt2](https://moveit.ros.org/install-moveit2/binary/) are installed, along with their dev tools.

## Installation
### Install dependency controller packages
`sudo apt install ros-humble-controller*`
`sudo apt-get install ros-humble-can-msgs`
`pip install catkin_pkg`

### Make sure everything is up to date
`sudo apt update` \
`sudo apt upgrade`

### Make a directory and clone the source code
`mkdir -p arctos/src`\
`cd arctos/src`\
`git clone https://github.com/coen132/arctos.git`

### Build the workspace
`cd ~/arctos`\
`colcon build --symlink-install`

This should run without any errors, otherwise make sure the dependencies are installed and sourced correctly.

## Sourcing the package in your bash script
After installation make sure to automatically source the package in your bash script, otherwise you need to do it everytime you run the package
Copy and paste the following at the bottom of the bash file: "source ~/arctos/install/local_setup.bash"

`sudo nano ~/.bashrc`

## Launching the simulation

To test out and make sure everything is correctly installed you can launch the demo simulation \
`cd ~/arctos`\
`source install/local_setup.bash`\
`ros2 launch arctos_moveit_config demo.launch.py` 

This should launch RVIZ and load the Arctos robot arm. 

