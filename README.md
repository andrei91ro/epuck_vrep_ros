# ROS extension of the e-puck simulated in V-REP

This script builds upon the e-puck model that comes bundled with [V-REP](http://www.v-rep.eu/) and uses the [V-REP ROSInterface plugin](http://www.coppeliarobotics.com/helpFiles/en/rosInterf.htm) in order to communicate with ROS.

The ROS topics are the same as those used by the [epuck\_driver\_cpp](https://github.com/gctronic/epuck_driver_cpp) ROS package, that is used to control a real robot.

This repository includes:

    * `epuck_vrep_ros.lua`: the lua script that can be attached to a simulated e-puck
    * `e-puck.ttm`: an epuck model that can be directly loaded in a V-REP scene, using `File -> Load model`

Before starting any simulation, make sure that the `opMode` parameter of the e-puck is to `2` as the first two values (`0` and `1`) are reserved for the original line-follower and leader-follow behaviours that came with the e-puck model. See the [V-REP user interface documentation](http://www.coppeliarobotics.com/helpFiles/en/userInterface.htm) for information on modifying parameters.
