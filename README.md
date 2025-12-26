# ur_prime2
This repo contains the source code for Gazebo based Prime2 project, using Universal Robot and Moveit2. Also featuring visual and force-torque servoing capability.


## Setup
### 1. Clone the repo
```
git clone https://github.com/PiusLim373/ur_prime2.git
git submodule update --init --recursive
```
### 2. Build
```
colcon build --symlink-install
```
### 3. Source the built environment, and export Gazebo resources path
```
source install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:<Path to working directory>/install/ur_description/share/
```

### 4. Add the force torque sensor gazebo plugin
Add in the following lines to `/usr/share/ignition/ignition-gazebo6/worlds/empty.sdf`, near the other plugins group
```
<plugin
    filename="ignition-gazebo-forcetorque-system"
    name="ignition::gazebo::systems::ForceTorque">
</plugin>
```
*I tried added to .urdf.xacro, but it is not working, this is my last option

## Run 
### 1. (First terminal) Start the simulation (Gazebo + Rviz)
```
ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur10e
```

### 2. (Second terminal) Start the vision, servo, master controllers
```
ros2 launch prime2_master prime2_master.launch.py 
```
### 3. (Third terminal) Spawn the FOUP
```
ros2 run ros_gz_sim create -file <Path to working directory>/src/Universal_Robots_ROS2_Description/meshes/prime2_obj/foup.sdf -allow_renaming true -name foup -x 0.78 -y 0.7 -z 1.98 -Y -1.49
```
*Adjust the spawn location accordingly
### 3. (Forth terminal) Start the actual sequence!
```
ros2 service call /run std_srvs/srv/Trigger {}
```
:warning: Remember to source the `install/` folder and export `IGN_GAZEBO_RESOURCE_PATH` for these terminals

## Video
[![Vision-Guided and Force-Torque Servoing for FOUP Handling on a Mobile Manipulator in Simulation](https://img.youtube.com/vi/R5OV3He6L50/0.jpg)](https://www.youtube.com/watch?v=R5OV3He6L50&feature=youtu.be)