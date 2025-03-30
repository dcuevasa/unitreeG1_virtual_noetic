# Introduction
Here are the ROS simulation packages for Unitree G1, You can load the robot and joint controllers in Gazebo, so you can perform low-level control (control the torque, position and angular velocity) of the robot joints. Please be aware that the Gazebo simulation cannot do high-level control, namely walking. Aside from these simulation functions, you can also control your real robots in ROS with the [unitree_ros_to_real](https://github.com/unitreerobotics/unitree_ros_to_real) packages. For real robots, you can do high-level and low-level control using our ROS packages.

## Packages:
Robot description:
* [`g1_description`](https://github.com/unitreerobotics/unitreeG1_virtual_noetic/tree/master/robots/g1_description)

Robot and joints controller:
* `unitree_controller`

Simulation related:
* `unitree_gazebo`
* `unitree_legged_control`

# Dependencies
* [ROS](https://www.ros.org/) Noetic
* [Gazebo11](http://gazebosim.org/)
* [unitree_legged_msgs](https://github.com/unitreerobotics/unitree_ros_to_real): `unitree_legged_msgs` is a package under [unitree_ros_to_real](https://github.com/unitreerobotics/unitree_ros_to_real).
# Build
<!-- If you would like to fully compile the `unitree_ros`, please run the following command to install relative packages. -->

For ROS Noetic:
```
mkdir -p unitree_ws/src
cd unitree_ws/src
git clone https://github.com/unitreerobotics/unitree_legged_sdk.git
git clone https://github.com/unitreerobotics/unitree_ros_to_real.git
git clone https://github.com/dcuevasa/unitreeG1_virtual_noetic
git clone https://github.com/fratopa/Mid360_simulation_plugin.git
# In case you are missing any of these
sudo apt-get update
sudo apt-get install ros-noetic-controller-interface ros-noetic-gazebo-ros-control ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-joint-trajectory-controller ros-noetic-robot-state-publisher
cd ..
catkin_make
source devel/setup.bash
# Launch your preferred simulation here
roslaunch unitree_gazebo empty_world.launch rname:=g1
```

If you face a dependency problem, you can just run `catkin_make` again.

# Detail of Packages
## unitree_legged_control:
It contains the joints controllers for Gazebo simulation, which allows users to control joints with position, velocity and torque. Refer to "[unitree_ros/unitree_controller/src/servo.cpp](https://github.com/unitreerobotics/unitree_ros/blob/master/unitree_controller/src/servo.cpp)" for joint control examples in different modes.

## The description of robots:
Namely the description of G1. Each package includes mesh, urdf and xacro files of robot. Take G1 for example, you can check the model in Rviz by:
```
roslaunch g1_description g1_rviz.launch
```

## unitree_gazebo & unitree_controller:
You can launch the Gazebo simulation with the following command:
```
roslaunch unitree_gazebo normal.launch rname:=g1 wname:=stairs
```
Where the `rname` means robot name, which can be `g1`. The `wname` means world name, which can be `earth`, `space` or `stairs`. And the default value of `rname` is `g1`, while the default value of `wname` is `earth`. In Gazebo, the robot should be lying on the ground with joints not activated.

### 1. Stand controller
After launching the gazebo simulation, you can start to control the robot:
```
rosrun unitree_controller unitree_servo
```

And you can add external disturbances, like a push or a kick:
```
rosrun unitree_controller unitree_external_force
```
### 2. Position and pose publisher
Here we demonstrated how to control the position and pose of robot without a controller, which should be useful in SLAM or visual development.

Then run the position and pose publisher in another terminal:
```
rosrun unitree_controller unitree_move_kinetic
```
The robot will turn around the origin, which is the movement under the world coordinate frame. And inside of the source file [move_publisher.cpp](https://github.com/unitreerobotics/unitree_ros/blob/master/unitree_controller/src/move_publisher.cpp), we also provide the method to move using the robot coordinate frame. You can change the value of `def_frame` to `coord::ROBOT` and run the catkin_make again, then the `unitree_move_publisher` will move robot under its own coordinate frame.