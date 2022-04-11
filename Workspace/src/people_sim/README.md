## About ##

This package contains scripts that publish a People topic in order
to simulate humans in an environment. This is used to test the
Social Navigation Layer of the cost maps.

## Usage ##

1. Launch the Gazebo world:

```sh
$ roslaunch neo_gazebo neobotix_factory_world.launch platform_configuration:=neo-cph
```

The 'platform_configuration' can also be set to `neo-aal`, however the lidar simulation
is less reliable which, for the purpose of testing, is not desirale.

2. Start RViz

```sh
$ roslaunch neo_2dnav 2dnav.launch
```

Note: A map is needed before launching the 'neo_2dnav' and the Social Navigation Layer
Plugin must be used as well.

3. Run the simuation

```sh
$ rosrun people_sim <simulation_script>.py
```
