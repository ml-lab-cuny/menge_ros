# MengeROS
MengeROS is a novel open-source crowd simulation tool for robot navigation that integrates [Menge](http://gamma.cs.unc.edu/Menge/) with ROS. It extends Menge to introduce one or more robot agents into a crowd of pedestrians. Each robot agent is controlled by external ROS-compatible controllers. MengeROS has been used to simulate crowds with up to 1000 pedestrians and 20 robots.

##  Contents
* **[Installation](#installation)**
* **[Robot Configuration](#robot-configuration)**
* **[Crowd Configuration](#crowd-configuration)**
* **[Examples](#examples)**
* **[Reference Paper](#reference-paper)**
* **[Contact](#contact)**
* **[Acknowledgments](#acknowledgments)**

## Installation
(Tested For Ubuntu 14.04 with ROS Indigo)
1. Download the source code into the catkin workspace of ROS.
2. Use `catkin_make` to recompile the ROS packages and then start roscore.
3. Run the simulator using the command:
~~~
rosrun menge_sim menge_sim -p examples\core\tradeshow.xml
~~~
4. The previous command should start the simulator with the tradeshow world and as many robots as specified in the `examples\core\tradeshow\tradeshowS.xml`. The various sample environments files included in MengeROS can be found in the examples folder in this repository.
5. Press space to start the simulation; pressing space again will pause the simulation.
6. The robot in the simulator can be controlled externally using any controller than sends [Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html) messages.

## Robot Configuration:
1. To introduce a new robot agent into MengeROS, update the specification file (`tradeshowS.xml`). Create a new `AgentProfile` and 
replace
~~~
\<Common class="2" r="0.26" external="0"/> 
~~~
with 
~~~
\<Common class="2" r="0.26" external="1" start_angle="-1.96" end_angle="1.918" increment="0.0005817" range_max="25"/>. 
~~~
The "external" variables ensure that the new agent is controlled externally. For more information on how to create an AgentProfile refer to the [Menge documentation](http://gamma.cs.unc.edu/Menge/files/mengeCDMain.pdf).

2. The radius of the robot can be configured by varying r=0.26. 
3. The laser range, the field of view and the number of ray traces can be set by changing `start_angle`, `end_angle`, `increment` and `range_max` ([See ROS documentation for more details](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html)). The configuration currently defaults to [Fetch robot specifications](http://docs.fetchrobotics.com/FetchAndFreight2016.pdf).
4. All distances are in meters and angles are in radians.

## Crowd Configuration:
1. The integration allows all Menge crowd configurations to be generated in MengeROS with no additional changes. The documentation on how to generate various crowd scenarios is given on the [Menge website](http://gamma.cs.unc.edu/Menge/docs/code/menge/html/).

## Examples:
Here are several examples of MengeROS simulations.  
1. This [video](https://youtu.be/Q-qEu4oBmsw) shows the tradeshow world in action with a single robot that is controlled by an [external controller](http://wiki.ros.org/teleop_twist_keyboard).

2. This [video](https://www.youtube.com/watch?v=zIs6h3l5YgU) compares ORCA and the original social forces model (Helbing, Dirk, and Peter Molnar. "Social Force Model for Pedestrian Dynamics." Physical Review E 51.5 (1995): 4282.). Other examples of different collision avoidance models for pedestrians can be found [here](http://gamma.cs.unc.edu/Menge/intro_vids.html).

3. This [video](https://www.youtube.com/watch?v=Ue1hHk6KlGg) shows an application of MengeROS for crowd-sensitive path planning. In the video, a robot (blue) navigates through a simple office-like environment around 90 pedestrians. The robot learns a distribution of the crowd using only local sensor observations. The left side shows the simulator; the right side shows the rviz visualization where the laser endpoints are in red. The dark regions of the grid indicate the likelihood of dense crowds.

## Reference paper
A paper describing MengeROS can be downloaded [here](http://www.cs.hunter.cuny.edu/~epstein/html/publications.html). If you are going to use this library for your work, please cite it within any resulting publication:

A. Aroor, S.L. Epstein, R. Korpan "MengeROS: a Crowd Simulation Tool for Autonomous Robot Navigation", AAAI 2017 Fall Symposium on Artificial Intelligence for Human-Robot Interaction, 2017.

The bibtex code for including this citation is provided:
~~~
@INPROCEEDINGS{aroor2017,
  AUTHOR={Aroor, Anoop  and  Epstein, Susan L  and  Korpan, Raj},
  TITLE={MengeROS: A Crowd Simulation Tool for Autonomous Robot Navigation},
  BOOKTITLE={AAAI 2017 Fall Symposium on Artificial Intelligence for Human-Robot Interaction},
  YEAR={2017}}
~~~

## Contact
Anoop Aroor
The Graduate Center of The City University of New York
aaroor@gradcenter.cuny.edu

## Acknowledgments
The development of MengeROS was supported in part by NSF Grant #1625843 and by the Machine Learning and Problem Solving Lab at Hunter College, CUNY.
