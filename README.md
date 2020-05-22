# A Framework for Multi-Robot Motion Planning, Coordination and Control 

This software implements an _online coordination method for multiple robots_. Its main features are:

* Goals can be posted and paths computed online
* Precedences are inferred online, accounting for robot dynamics via provided dynamic models
* Very few assumptions are made on robot controllers
* The coordination method is not specific to a particular motion planning technique

The software includes a basic 2D robot simulation and a simple built-in motion planner (which depends on the <a href="http://ompl.kavrakilab.org/">OMPL</a> library). A <a href="https://github.com/FedericoPecora/coordination_oru_ros">separate interface package</a> is provided to enable the use of this software in conjunction with <a href="http://www.ros.org/">ROS</a> and the <a href="https://github.com/OrebroUniversity/navigation_oru-release">navigation_oru</a> stack to obtain a fully implemented stack for multi-robot coordination and motion planning.

## Overview
The algorithm provided by this implementation is detailed in

* Federico Pecora, Henrik Andreasson, Masoumeh Mansouri, and Vilian Petkov, <a href="https://www.aaai.org/ocs/index.php/ICAPS/ICAPS18/paper/view/17746/16941">A loosely-coupled approach for multi-robot coordination, motion planning and control</a>. In Proc. of the International Conference on Automated Planning and Scheduling (ICAPS), 2018.

[![Examples usages of the coordination_oru library](http://img.youtube.com/vi/jCgrCVWf8sE/0.jpg)](http://www.youtube.com/watch?v=jCgrCVWf8sE "Examples usages of the coordination_oru library")

The approach makes very few assumptions on robot controllers, and can be used with any motion planning method for computing kinematically-feasible paths. Coordination is seen as a high-level control scheme for the entire fleet. Heuristics are used to update precedences of robots through critical sections while the fleet is in motion, and the dynamic feasibility of precedences is guaranteed via the inclusion of user-definable models of robot dynamics. 

The coordination method is based on the _trajectory envelope_ representation provided by the <a href="http://metacsp.org">Meta-CSP framework</a>. This representation is detailed in

* Federico Pecora, Marcello Cirillo, Dimitar Dimitrov, <a href="http://ieeexplore.ieee.org/abstract/document/6385862/">On Mission-Dependent Coordination of Multiple Vehicles under Spatial and Temporal Constraints</a>, IEEE/RSJ International Conference on Intelligent Robots and Systems (2012), pp. 5262-5269.

In short, a trajectory envelope is a set of spatio-temporal constraints on a robot's trajectory. A trajectory envelope spans over a _path_, which is a sequence of _poses_ ```<p1, ... pn>```. In the current implementation, the spatial constraints defining a trajectory envelope are computed as the sweep of the robot's footprint over the path.

## Tutorial
The approach is discussed in detail in the tutorial on _Integrated Motion Planning, Coordination and Control for Fleets of Mobile Robots_, given at the <a href="http://icaps18.icaps-conference.org/tutorials/">2018 International Conference on Automated Planning and Scheduling (ICAPS)</a> by F. Pecora and M. Mansouri. Slides and source code of the tutorial are available <a href="https://gitsvn-nt.oru.se/fopa/coordination-tutorial-src-ICAPS-2018">here</a>.

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
In the example ```TestTrajectoryEnvelopeCoordinatorThreeRobots```, missions are continuously posted for three robots to reach locations along intersecting paths. The paths are stored in files provided in the ```paths``` directory. The poses of locations and pointers to relevant path files between locations are stored in the self-explanatory ```paths/test_poses_and_path_data.txt``` file.

## Visualizations
The API provides three visualization methods:

* ```BrowserVisualization```: a browser-based visualization.
* ```JTSDrawingPanelVisualization```: a Swing-based visualization.
* ```RVizVisualization```: a visualization based on the ROS visualization tool <a href="http://wiki.ros.org/rviz">RViz</a>.

All three visualizations implement the abstract ```FleetVisualization``` class, which can be used as a basis to create your own visualization.

Most examples use the ```BrowserVisualization```. The state of the fleet can be viewed from a browser at <a href="http://localhost:8080">http://localhost:8080</a>. The image below shows this visualization for the ```TestTrajectoryEnvelopeCoordinatorThreeRobots``` example:

![BrowserVisualization GUI](images/browser-gui.png "Browser-based visualization")

An arrow between two robots indicates that the source robot will yield to the target robot. Priorities are computed based on a heuristic (which can be provided by the user) and a forward model of robot dynamics (which can also be provided, and is assumed to be conservative - see the <a href="http://iliad-project.eu/wp-content/uploads/papers/PecoraEtAlICAPS2018.pdf">ICAPS 2018 paper</a> mentioned above). The specific poses at which robots yield are also updated online, based on the current positions of robots and the intersecting areas of their trajectory envelopes (critical sections). This makes it possible to achieve "following" behavior, that is, the yielding pose of a robot is updated online while the "leading" robot drives.

The a Swing-based GUI provided by class ```JTSDrawingPanelVisualization``` looks like this:

![Swing-based GUI](images/coord.png "Swing-based visualization")

This GUI allows to take screenshots in SVG, EPS and PDF formats by pressing the ```s```, ```e``` and ```p``` keys, respectively (while focus is on the GUI window). Screenshots are saved in files named with a timestamp, e.g., ```2017-08-13-11:13:17:528.svg```. Note that saving PDF and EPS files is computationally demanding and will temporarily interrupt the rendering of robot movements; SVG screenshots are saved much quicker.

The ```RVizVisualization``` visualization publishes <a href="http://wiki.ros.org/rviz/DisplayTypes/Marker">visualization markers</a> that can be visualized in <a href="http://wiki.ros.org/rviz">RViz</a>. The class also provides the static method ```writeRVizConfigFile(int ... robotIDs)``` for writing an appropriate RViz confiuration file for a given set of robots. An example of the visualization is shown below.

![RVizVisualization GUI](images/rviz-gui.png "RViz-based visualization")

The visualization with least computational overhead is the ```RVizVisualization```, and is recommended for fleets of many robots. The ```BrowserVisualization``` class serves an HTML page with a Javascript which communicates with the coordinator via websockets. Although rendering in this solution is less efficient than in RViz, the rendering occurs on the client platform (where the browser is running), so its computational overhead does not necessarily affect the coordination algorithm. The ```JTSDrawingPanelVisualization``` is rather slow and not recommended for fleets of more than a handful of robots, however it is practical (not requiring to start another process/program for visualization) and relatively well-tested.

## Logging

More detailed information about execution is posted in the terminal and saved to log files. Log files can be inspected offline by running class ```coordination_oru.util.BrowseLogs```, which opens a log browsing GUI. Each panel in the GUI shows the output of one of the class instances that ran in the previous execution of the test program. Several of these classes are instantiated in separate threads, and messages produced concurrently are highlighted when the caret position in one of the panels is updated by the user. The key-bindings Alt-\<X\> and Ctrl-Alt-\<X\> can be used to quickly select panel \<X\> in the top and bottom pane, respectively.  

![LogBrowser GUI](images/logs.png "LogBrowser GUI")

## The ```SimpleReedsSheppCarPlanner``` motion planner

A simple motion planner is provided for testing the coordination framework without the need for pre-computed path files. The planner can be used to obtain paths for robots with Reeds-Shepp kinematics (Dubin's car-like robots that can move both forwards and backwards), and is used in several of the included demos.

The provided motion planner depends on the <a href="http://ompl.kavrakilab.org/">Open Motion Planning Library (OMPL)</a>. The motion planner and its Java interface are purposefully kept very simple. It performs rather poorly in terms of the quality of paths it returns, and is _not_ suited for anything beyond simple examples. Please consider developing a more performing and principled integration with your motion planning software of choice, as done in the <a href="https://github.com/FedericoPecora/coordination_oru_ros">coordination_oru_ros</a> package.

## Installing the ```SimpleReedsSheppCarPlanner``` motion planner

Please install the OMPL library, which is available in the official Ubuntu repositories:

```
$ sudo apt install libompl-dev
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

## Sponsors
This project is supported by

* The <a href="http://semanticrobots.oru.se">Semantic Robots</a> Research Profile, funded by the <a href="http://www.kks.se/">Swedish Knowledge Foundation</a>
* The <a href="https://iliad-project.eu/">ILIAD Project</a>, funded by the <a href="https://ec.europa.eu/programmes/horizon2020/">EC H2020 Program</a>
* The iQMobility Project, funded by <a href="https://www.vinnova.se/">Vinnova</a>

## License
coordination_oru - Robot-agnostic online coordination for multiple robots

Copyright &copy; 2017-2020 Federico Pecora

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.
