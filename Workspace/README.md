#### Table of contents
- [Simulation of Navigation Stack (Navigation)](#Simulation-of-navigation-stack-navigation)
- [Simulation of Navigation Stack (SLAM)](#Simulation-of-navigation-stack-SLAM)
- [Plugin](#Plugin)




## Simulation of Navigation Stack (Navigation)

- Launch Gazebo with loaded robot model and world.
```c
roslaunch neo_simulations {launch_file_name}
```

- Launch Navigation Stack 
```c
    roslaunch neo_2dnav 2dnav.launch
```
<br>

## Simulation of Navigation Stack (SLAM)
- Launch Gazebo with loaded robot model and world.
```c
roslaunch neo_simulations {launch_file_name}
```

- Launch SLAM
```c
    roslaunch neo_2dnav nav2dslam.launch
```
<br>

## Plugin
The plugin should be installed when downloading the repository. To tell Gazebo where to find the plugin, insert this line to the terminal (with your own workspace path instead of `{ws_path}`) 

```c
echo 'export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:{ws_path}/actor_collisions/build' >> ~/.bashrc
``` 

Restart the terminal or run this command `. ~/.bashrc` 

<br>
