# Package for depicting the robot path using a projector #

This package uses the AAXA P450 Micro Projector, but can be configured to any projector.

If the placement of the projector is changed, remember to adjust it in the .launch files in **neobotix_mp_655**.

The Rviz camera info also needs to be adjusted to fit the projector.

## Running the package ##

For displaying the path on the projector, run **projector.launch** when the planner is running.

```sh
$ roslaunch arrow_projection projector.launch
```

Remember to make the image full screen on the projector.