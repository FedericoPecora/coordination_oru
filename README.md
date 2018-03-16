# coordination_oru

This software implements an _online coordination method for multiple robots_. Its main features are:

* Goals can be posted and paths computed online
* Precedences are inferred online, accounting for robot dynamics via provided dynamic models
* Very few assumptions are made on robot controllers
* The coordination method is not specific to a particular motion planning technique

The software includes a basic 2D robot simulation and a simple built-in motion planner (which depends on the <a href="http://ompl.kavrakilab.org/">OMPL</a> and <a href="http://www.mrpt.org/">MRPT</a> libraries). A <a href="https://github.com/FedericoPecora/coordination_oru_ros">separate interface package</a> is provided to enable the use of this software in conjunction with <a href="http://www.ros.org/">ROS</a> and the <a href="https://github.com/OrebroUniversity/navigation_oru-release">navigation_oru</a> stack to obtain a fully implemented stack for multi-robot coordination and motion planning.

## Overview
The algorithm provided by this implementation is detailed in

* Federico Pecora, Henrik Andreasson, Masoumeh Mansouri, and Vilian Petkov, <a href="">A loosely-coupled approach for multi-robot coordination, motion planning and control</a>. In Proc. of the International Conference on Automated Planning and Scheduling (ICAPS), 2018.

[![Examples usages of the coordination_oru library](http://img.youtube.com/vi/jCgrCVWf8sE/0.jpg)](http://www.youtube.com/watch?v=jCgrCVWf8sE "Examples usages of the coordination_oru library")

The approach makes very few assumptions on robot controllers, and can be used with any motion planning method for computing kinematically-feasible paths. The overall fleet automation problem is seen as a high-level control scheme for the entire fleet. Heuristics are used to update precedences of robots through critical sections while the fleet is in motion, and the dynamic feasibility of precedences is guaranteed via the inclusion of user-definable models of robot dynamics. 

The coordination method is based on the _trajectory envelope_ representation provided by the <a href="http://metacsp.org">Meta-CSP framework</a>. This representation is detailed in

* Federico Pecora, Marcello Cirillo, Dimitar Dimitrov, <a href="http://ieeexplore.ieee.org/abstract/document/6385862/">On Mission-Dependent Coordination of Multiple Vehicles under Spatial and Temporal Constraints</a>, IEEE/RSJ International Conference on Intelligent Robots and Systems (2012), pp. 5262-5269.

In short, a trajectory envelope is a set of spatio-temporal constraints on a robot's trajectory. A trajectory envelope spans over a _path_, which is a sequence of _poses_ ```<p1, ... pn>```. In the current implementation, the spatial constraints defining a trajectory enevlope are computed as the sweep of the robot's footprint over the path.

## Installation
To install, clone this repository and compile the source code with gradle (redistributable included):

```
$ git clone https://github.com/FedericoPecora/coordination_oru.git
$ cd coordination_oru
$ ./gradlew install
```

## Running an example
A number of examples are provided. Issue the following command from the source code root directory for instructions on how to run the examples:
```
$ ./gradlew run
```
In the first demo, ```TestTrajectoryEnvelopeCoordinatorThreeRobots```, missions are continuously posted for three robots to reach locations along intersecting paths. The paths are stored in files provided in the ```paths``` directory. The poses of locations and pointers to relevant path files between locations are stored in the self-explanatory ```paths/test_poses_and_path_data.txt``` file.

Running the examples opens a GUI showing the current state of the three robots. For the first example, the GUI looks like this:

![Coordination GUI](images/coord.png "Coordination GUI")

A gray arrow between two robots indicates that the source robot will yield to the target robot. Priorities are computed every time a new mission is added. If multiple missions are added in batch, yielding behavior follows a fixed priority which can be specified programmatically. Driving robots always have priority over robots whose missions have been newly added. The specific poses at which robots yield are computed on the fly based on the current positions of robots and the intersecting areas of their trajectory envelopes (critical sections). This makes it possible to achieve "following" behavior, that is, the yielding pose of a robot is updated online while the "leading" robot drives.

The GUI also allows to take screenshots in SVG, EPS and PDF formats by pressing the ```s```, ```e``` and ```p``` keys, respectively (while focus is on the GUI window). Screenshots are saved in files named with a timestamp, e.g., ```2017-08-13-11:13:17:528.svg```. Note that saving PDF and EPS files is computationally demanding and will temporarily interrupt the rendering of robot movements; SVG screenshots are saved much quicker.

More detailed information about execution is posted in the terminal and saved to log files. Log files can be inspected offline by running class ```coordination_oru.util.BrowseLogs```, which opens a log browsing GUI. Each panel in the GUI shows the output of one of the class instances that ran in the previous execution of the test program. Several of these classes are instantiated in separate threads, and messages produced concurrently are highlighted when the caret position in one of the panels is updated by the user. The key-bindings Alt-\<X\> and Ctrl-Alt-\<X\> can be used to quickly select panel \<X\> in the top and bottom pane, respectively.  

![LogBrowser GUI](images/logs.png "LogBrowser GUI")

## The ```SimpleReedsSheppCarPlanner``` motion planner

A simple motion planner is provided for testing the coordination framework without the need for pre-computed path files. The planner can be used to obtain paths for robots with Reeds-Shepp kinematics (Dubin's car-like robots that can move both forwards and backwards), and is used in several of the included demos.

The provided motion planner depends on

* The Open Motion Planning Library (OMPL), http://ompl.kavrakilab.org/

* The Mobile Robot Programming Toolkit (MRPT), http://www.mrpt.org/

The motion planner and its Java interface are purposefully kept very simple. It performs rather poorly in terms of the quality of paths it returns, and is _not_ suited for anything beyond simple examples. Please consider developing a more performing and principled integration with your motion planning software of choice, as done in the <a href="https://github.com/FedericoPecora/coordination_oru_ros">coordination_oru_ros</a> package.

## Installing the ```SimpleReedsSheppCarPlanner``` motion planner

Please install the OMPL and MRPT libraries. Both are present in the official Ubuntu repositories (tested on Ubuntu 16.04):

```
$ sudo apt-get install libompl-dev
$ sudo apt-get install mrpt-apps libmrpt-dev
```

Then, compile and install the ```simplereedssheppcarplanner``` shared library as follows:

```
$ cd coordination_oru/SimpleReedsSheppCarPlanner
$ cmake .
$ make
$ sudo make install
$ sudo ldconfig
```

This will install ```libsimplereedssheppcarplanner.so``` in your ```/usr/local/lib``` directory. A simple JNA-based Java interface to the library is provided in package ```se.oru.coordination.coordination_oru.motionplanning```. The Java class  ```ReedsSheppCarPlanner``` in the same package can be instantiated and used to obtain motions for robots with Reeds-Shepp kinematics.

## Using the ```SimpleReedsSheppCarPlanner``` motion planner

A simple example showing how to invoke the motion planner is provided by class ```TestReedsSheppCarPlanner``` in package ```se.oru.coordination.coordination_oru.motionplanning.tests```.

Most of the coordination examples make use of the motion planner (see screenshot below). Issue command

```$ ./gradlew run```

for a list of all provided examples and instructions on how to run them (and/or see package ```se.oru.coordination.coordination_oru.tests```).

![Coordination with the ReedsSheppCarPlanner](images/coord-rsp.png "Coordination with the ReedsSheppCarPlanner")

## License
coordination_oru - Online coordination for multiple robots

Copyright (C) 2017-2018 Federico Pecora

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.
