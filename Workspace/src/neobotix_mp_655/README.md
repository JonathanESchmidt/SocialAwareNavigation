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