# Content

Packages overview
* `neo_2dnav`: mapping, slam, navigation
* `neo_bringup`: launch hardware drivers for the physical robots
* `neo_description`: URDF/XACRO models
* `neo_msgs`: Neobotix specific ROS message types
* `neo_platformctrl_diff`: transformation node for neobotix robots, kinematics for 2-wheel diff robot
* `neo_relayboard`: node for the relayboard on the Neobotix platforms
* `neo_teleop`: keyboard and teleoperation node for moving the robot

`cob_driver` packages: https://github.com/ipa320/cob_driver
* `cob_scan_unifier`: combines several laser scanners into one scan data (this is an old/custom version. The one through *apt* does not work with our implementation at the moment)
* `cob_sick_s300`: publishes a laser scan message out of a Sick S300 laser scanner (this is an old/custom version. The one through *apt* does not work with our implementation at the moment)

# Platform Bring-Up

The system is made up of several distinct systems connected via Ethernet cable. These are:
- MiR platform
- Nvidia Jetson
- Machine running the navigation stack (in our case a laptop mounted on the MiR)

The following steps need to be taken in order to bring the entire system online:

### 1. MiR Platform

Firstly, the platform needs to be turned on. This is achieved by turning the ON switch on the robot itself. Wait for around a minute to ensure that the WiFi router is fully online.
Secondly, the MiR platform interface needs to be started. This is done by using the following launch file on the machine mounted on the MiR:
```
roslaunch neo_2dnav mir_bringup.launch
```
This might take a few seconds. It has completed when the launch prints the list of topics in the terminal.

### 2. Nvidia Jetson

The CNN used for the people localiser is running on the Nvidia Jetson board and the launch files are found in a separate catkin workspace which needs to be made and sourced.

NOTE: This is to be run on the Nvidia Jetson. Thus it is necessary to ssh into the board and then run the commands below.

```
cd /SocialAwareNavigation/JetsonWorkspace/
catkin_make
source devel/setup.bash
```
Once the build is done, run the people localiser launch file:

```
roslaunch people_detection people_localiser.launch
```

### 3. Launching the navigation stack

Once the MiR platform and people localiser systems are online, the navigation stack (which also includes the video projector) can be started. This is to done on the machine (laptop) mounted on the robot. The command to run the navigation stack is:
```
roslaunch neo_2dnav mason_nav_mir.launch
```

### Environment details

As there are nodes running on different devices connected via Ethernet, great care needs to be taken when exporting the **ROS_MASTER_URI** and **ROS_HOSTNAME** in the ~/.bashrc files on each device. 