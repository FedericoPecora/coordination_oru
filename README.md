# coordination_oru
This package provides an online coordination method for multiple robots. It is based on the Meta-CSP Framework library available at <a href="http://metacsp.org">metacsp.org</a>. This package includes a basic 2D robot simulation, and does not require ROS or ROSJava. To use this package with <a href="http://www.ros.org">ROS</a> and the <a href="https://github.com/OrebroUniversity/navigation_oru-release">navigation_oru</a> stack, please see <a href="https://github.com/FedericoPecora/coordination_oru_ros">coordination_oru_ros</a>.

## Overview
The coordination method is based on the trajectory envelope representation provided by the Meta-CSP framework. A _trajectory envelope_ is a set of spatio-temporal constraints on a robot's trajectory. A trajectory envelope spans over a _path_, which is a sequence of _poses_ ```<p1, ... pn>```. In the current implementation, the spatial constraints defining a trajectory enevlope are computed as the sweep of the robot's footprint over the path. For more details on trajectory envelopes, please see

* Federico Pecora, Marcelo Cirillo, Dimitar Dimitrov, <a href="http://ieeexplore.ieee.org/abstract/document/6385862/">On Mission-Dependent Coordination of Multiple Vehicles under Spatial and Temporal Constraints</a>, IEEE/RSJ International Conference on Intelligent Robots and Systems (2012), pp. 5262-5269.

The coordination algorithm provided in this implementation works as follows:

1. For each pair of trajectory envelopes (```te1```, ```te2```) that are navigated by robots ```R1``` and ```R2```, compute the areas of spatial intersection of the trajectory envelopes. Each such contiguous area is a _critical section_, defined as a tuple (```te1```, ```te2```, [```start1```, ```end1```], [```start2```, ```end2```]), where

   * ```te1``` and ```te2``` are trajectory envelopes that intesect in the critical section
   
   * ```start1``` (```start2```) is the index of the first pose along the path navigated by robot ```R1``` (```R2```) at which this robot's footprint intersects ```te2``` (```te1```)
   
   * ```end1``` (```end2```) is the index of the first pose beyond ```start1``` (```start2```) along the path navigated by robot ```R1``` (```R2```) from which this robot's footprint ceases to intersect ```te2``` (```te1```)

2. For each critical section (```te1```, ```te2```, [```start1```, ```start2```], [```end1```, ```end2```]) found at step 1, decide whether the critical section is relevant, and, if so, which among robots ```{ R1, R2 }``` should transit the critical section first:

   * if either of the two robots has exited the critical section, discard this critical section
   
   * if neither robot has entered the critical section, assign priority to the robot that is closest to the start of the critical section

   * if only one of the two robots has entered the critical section, assign priority to that robot

   * (note that a priority has already been assigned if both robots have entered the critical section)

3. For each critical section (```te1```, ```te2```, [```start1```, ```start2```], [```end1```, ```end2```]) found by step 1 and not discarded by step 2, inform the robot that was _not_ given priority in step 2 (let this be ```R2```) that it _cannot_ proceed beyond a _critical point_ ```p``` defined as max(```start2```, ```start2``` + (```cp1``` - ```start1```) - ```s```), where

   * ```cp1``` is the index of the pose in the path of ```te1``` that is closest to the current pose of the robot given priority (```R1```)
   
   * ```s``` is a safety distance (number of path indices by which ```R2``` should stay behind ```R1```)

New critical sections are added (step 1) whenever new missions are added. Critical sections are filtered and priorities decided (step 2), and critical points for each robot are updated (step 3) at a specified control period (by default, 1000 msec).

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

![alt text](images/coord.png "Coordination GUI")

A gray arrow between two robots indicates that the source robot will yield to the target robot. Priorities are computed every time a new mission is added. If multiple missions are added in batch, yielding behavior follows a fixed priority which can be specified programmatically. Driving robots always have priority over robots whose missions have been newly added. The specific poses at which robots yield are computed on the fly based on the current positions of robots and the intersecting areas of their trajectory envelopes (critical sections). This makes it possible to achieve "following" behavior, that is, the yielding pose of a robot is updated online while the "leading" robot drives.

The GUI also allows to take screenshots in SVG, EPS and PDF formats by pressing the ```s```, ```e``` and ```p``` keys, respectively (while focus is on the GUI window). Screenshots are saved in files named with a timestamp, e.g., ```2017-08-13-11:13:17:528.svg```. Note that saving PDF and EPS files is computationally demanding and will temporarily interrupt the rendering of robot movements; SVG screenshots are saved much quicker.

More detailed information about execution is posted in the terminal and saved to log files. Log files can be inspected offline by running class ```coordination_oru.util.BrowseLogs```, which opens a log browsing GUI. Each panel in the GUI shows the output of one of the class instances that ran in the previous execution of the test program. Several of these classes are instantiated in separate threads, and messages produced concurrently are highlighted when the caret position in one of the panels is updated by the user. The key-bindings Alt-\<X\> and Ctrl-Alt-\<X\> can be used to quickly select panel \<X\> in the top and bottom pane, respectively.  

![alt text](images/logs.png "LogBrowser GUI")

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

![alt text](images/coord-rsp.png "Coordination with the ReedsSheppCarPlanner")

## License
coordination_oru - Online coordination for multiple robots

Copyright (C) 2017 Federico Pecora

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.
