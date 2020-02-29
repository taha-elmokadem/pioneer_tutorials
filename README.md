# pioneer_tutorials

A set of instructions and tutorials are provided to perform simulations in [Gazebo](http://gazebosim.org/) using the Pioneer 3Dx model.

The recommended workstation setup is
* System: Ubuntu 16.04 LTS
* ROS distro: Kinetic

## ROS Installation Instructions (Ubuntu)
1. Install [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
    - Setup your sources.list and keys
    ```
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    ```
    - Installation of ROS-Desktop-Full Install: (Recommended) which includes  ROS, rqt, rviz, robot-generic libraries, Gazebo7, etc.
    ```
    sudo apt-get update
    sudo apt-get install ros-kinetic-desktop-full
    ```
    - Initialize rosdep
    ```
    sudo rosdep init
    rosdep update
    ```
    - Environment setup
    ```
    echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```
    - Dependencies for building packages
    ```
    sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
    ```
2. Install [catkin_tools](http://wiki.ros.org/kinetic/Installation/Ubuntu) (Recommended over [catkin_make](http://wiki.ros.org/catkin/commands/catkin_make))
    ```
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
    wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
    sudo apt-get update
    sudo apt-get install python-catkin-tools
    ```
3. Setup a catkin workspace (if not done already):
    ```
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws
    catkin init
    ```
4. Use [catkin_build](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html) to build packages
    ```
    cd ~/catkin_ws
    catkin build
    ```
* Note that ```catkin build``` is another build tool than ```catkin_make```. Make sure you will only use one method. In case you want to switch between the two tools, you need to delete the build directory first by running:
    ```
        cd ~/catkin_ws
        sudo rm -r build
        sudo rm -r devel
        sudo rm -r logs
    ```
5. Install [Git]() which is needed to install some packages from source
    ```
    sudo apt-get update
    sudo apt-get install git-core
    ```
## Gazebo Installation

- Gazebo 7 is installed by default with ROS-Desktop-Full installation. It is the recmmended version for ROS Kinetic
- Make sure you have the ROS packages "ros-control" and "ros-controllers" installed
    ```
    sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers
    ```

## Pioneer_Gazebo_ROS Installation

```
cd ~/catkin_ws/src
git clone https://github.com/JenJenChung/pioneer_gazebo_ros.git
git clone https://github.com/JenJenChung/pioneer_description.git
git clone https://github.com/JenJenChung/pioneer_2dnav.git
git clone https://github.com/JenJenChung/nav_bundle.git
git clone https://github.com/JenJenChung/simple_navigation_goals.git
catkin build
```
## Running single robot simulations
- using a single Pioneer 3dx robot with an empty world
    1. Modify the file ```~/catkin_ws/src/pioneer_gazebo_ros/pioneer_gazebo/launch/pioneer_world.launch``` to include the preferred world (environment) for simulation. To start with an empty world with no obstacles, replace the following line
        ```
        <arg name="world_name" value="$(find pioneer_gazebo)/worlds/utm_0.world"/>
        ```
        with
        ```
        <arg name="world_name" value="$(find pioneer_gazebo)/worlds/blank.world"/>
        ```

        Note: for a list of available worlds files, check ```~/catkin_ws/src/pioneer_gazebo_ros/pioneer_gazebo/worlds```
    2. Run the simulation and control interface:
        ```
            roslaunch pioneer_gazebo pioneer_world.launch && roslaunch pioneer_ros pioneer_controller_spin_recover.launch
        ```
    3. Odometery (position and velocity) of the robot is being published over the topic ```/odom``` of type [nav_msgs/Odometry](http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html)

    4. Velocity commands can be sent to the robot using the topic ```/pioneer/cmd_vel``` of type [geometry_msgs/Twist](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html)

## Running multiple robots simulations
- using multiple Pioneer 3dx robots in an empty world
    1. Modify the file ```~/catkin_ws/src/pioneer_gazebo_ros/pioneer_gazebo/launch/pioneer_world.launch``` to include the preferred world (environment) for simulation. To start with an empty world with no obstacles, replace the following line
        ```
        <arg name="world_name" value="$(find pioneer_gazebo)/worlds/utm_0.world"/>
        ```
        with
        ```
        <arg name="world_name" value="$(find pioneer_gazebo)/worlds/blank.world"/>
        ```

        Note: for a list of available worlds files, check ```~/catkin_ws/src/pioneer_gazebo_ros/pioneer_gazebo/worlds```
    2. Run the simulation, spawn vehicles at random initial positions, and run control interface:
        ```
        cd ~/catkin_ws/src/pioneer_gazebo_ros
        ./run_multi_pioneer
        ```
    3. Odometery topic and velocity commands of each robot will have a namesapce, for example, ```/pioneer1/odom``` and ```/pioneer1/pioneer/cmd_vel```

## Resources

* [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)
* [Gazebo Tutorials](http://gazebosim.org/tutorials)
* [Moving The Pioneer 3-DX In Gazebo](http://wiki.lofarolabs.com/index.php/Moving_The_Pioneer_3-DX_In_Gazebo)
* [Gazebo and Pioneer AT](https://wiki.nps.edu/display/RC/Gazebo+and+Pioneer+AT)
* [ROS kinetic and Gazebo7 Interface for the Pioneer3dx Simulation](http://jenjenchung.github.io/anthropomorphic/Code/Pioneer3dx%20simulation/ros-kinetic-gazebo7-pioneer.pdf)
* [AMR Pioneer Info](http://wiki.ros.org/action/show/Robots/AMR_Pioneer_Compatible?action=show&redirect=Robots%2FPioneer)