# ROS extension of the e-puck simulated in V-REP

This script builds upon the e-puck model that comes bundled with [V-REP](http://www.v-rep.eu/) and uses the [V-REP ROSInterface plugin](http://www.coppeliarobotics.com/helpFiles/en/rosInterf.htm) in order to communicate with ROS.

The ROS topics are the same as those used by the [epuck\_driver\_cpp](https://github.com/gctronic/epuck_driver_cpp) ROS package, that is used to control a real robot, namely:

Input from robot:

* `proximity0` to `proximity7`
* `camera`
* `imu`
* `tf`

Output to robot:

* `cmd_vel`
* `cmd_led`

The topics are prefixed with the robot number, e.g `/ePuck0/camera`.
Copy-pasted robots will be numbered incrementaly.

This repository includes:

    * `epuck_vrep_ros.lua`: the lua script that can be attached to a simulated e-puck
    * `e-puck.ttm`: an epuck model that can be directly loaded in a V-REP scene, using `File -> Load model`

Before starting any simulation, make sure that the `opMode` parameter of the e-puck is to `2` as the first two values (`0` and `1`) are reserved for the original line-follower and leader-follow behaviours that came with the e-puck model. See the [V-REP user interface documentation](http://www.coppeliarobotics.com/helpFiles/en/userInterface.htm) for information on modifying parameters.

## Detailed description

A detailed description of this package and the noise model that was applied in order to simulate infrared sensor noise is avaialble in the following paper:

[A. G. Florea, “Integrating a V-Rep Simulated Mobile Robot into Ros,” Univ. Politeh. Buchar. Sci. Bull. Ser. C-Electr. Eng. Comput. Sci., vol. 80, no. 3, pp. 3–16, 2018](https://www.scientificbulletin.upb.ro/rev_docs_arhiva/rez72b_106350.pdf)


## Usage

Assuming you have a working ROS and VREP configuration, complete with the `epuck_driver_cpp`, using this extension is simply a matter of drag-and-dropping `e-puck.ttm` into your V-REP scene.

## Easy start - Docker

The following is usefull only to users that do not have a working ROS, VREP configuration.

In order to simplify the installation procedure, users of Docker can use a [specially built container](https://hub.docker.com/r/naturo/epuck_vrep_ros/) that has ROS, V-REP and this extension pre-installed.

After installing docker, the container can be downloaded using:
`docker pull naturo/epuck_vrep_ros`

Once downloaded, the steps required to simulate an e-puck in V-REP and control it through ROS are shown on the DockerHub page:
[https://hub.docker.com/r/naturo/epuck\_vrep\_ros/](https://hub.docker.com/r/naturo/epuck_vrep_ros/)

# License notice

The MIT license applies specifically to the ROS extensions introduced in this repository.
The rest of the code is licensed under the [e-puck](http://www.e-puck.org/) license.

If this extension was useful in your academic work, please cite the following paper:

A. G. Florea, “Integrating a V-Rep Simulated Mobile Robot into Ros,” Univ. Politeh. Buchar. Sci. Bull. Ser. C-Electr. Eng. Comput. Sci., vol. 80, no. 3, pp. 3–16, 2018.

```bibtex
@article{ ISI:000440896700001,
    Author = {Florea, Andrei George},
    Title = {{INTEGRATING A V-REP SIMULATED MOBILE ROBOT INTO ROS}},
    Journal = {{UNIVERSITY POLITEHNICA OF BUCHAREST SCIENTIFIC BULLETIN SERIES
       C-ELECTRICAL ENGINEERING AND COMPUTER SCIENCE}},
    Year = {{2018}},
    Volume = {{80}},
    Number = {{3}},
    Pages = {{3-16}},
    Publisher = {{POLYTECHNIC UNIV BUCHAREST}},
    Address = {{SPLAIUL INDEPENDENTEI 313, SECTOR 6, BUCH, 060042, ROMANIA}},
    Type = {{Article}},
    Language = {{English}},
    ISSN = {{2286-3540}},
    Keywords = {{simulation; V-REP; ROS; e-puck}},
}
```

# Authors
Andrei George Florea, [Cătălin Buiu](http://catalin.buiu.net)

[Department of Automatic Control And Systems Engineering](http://acse.pub.ro),

Politehnica University of Bucharest

Bucharest, Romania.
