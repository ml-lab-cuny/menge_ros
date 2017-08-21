# menge_ros
Crowd simulation tool for robot navigation based on Menge(http://gamma.cs.unc.edu/Menge/) and ROS.
It extends Menge to introduce a robot agent into the crowd and which can be controller by external ROS compatible controllers. In other worlds now menge becomes a combined simulator for both crowd and robots.

Installation Guidelines: (Tested For Ubuntu 14.04 with ROS Indigo)

1. Download the source code into the catkin workspace of ROS.
2. Use catkin\_make to recompile the ROS packages. Start roscore.
3. RUN the simulator using the command rosrun menge\_sim menge\_sim -p examples\core\tradeshow.xml
4. The previous command should start the simulator with tradeshow world with as many robots as specified in the examples\core\tradeshow\tradeshowS.xml, the sample world files can be found the menge website or in examples.tar.gz in this repository.
5. Press space to start the simulation, pressing space again will pause the simulation
6. The robot in the simulator can be controlled externally using any controller than sends http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html messages


Configuration:

1. To introduce a new robot agent into menge, update the specification file (tradeshowS.xml). Create a new AgentProfile and 
replace  "<Common class="2" r="0.26" external="0"/>" with "<Common class="2" r="0.26" external="1" start_angle=-1.96 end_angle=1.918 increment=0.0005817 range_max=25/>". The "external" variables ensures that the new agent is controlled externally.
2. The radius of the robot can be configures by varying r=0.26. 
3. The laser range, the field of view and the number of scanners can be set by changing start_angle, end_angle, increment and range_max. Currently set to Fetch robot specifications. (http://docs.fetchrobotics.com/FetchAndFreight2016.pdf)

Examples:
1. https://youtu.be/Q-qEu4oBmsw

The video shows the tradeshow world in action with a single robot that is controlled by a external controller (http://wiki.ros.org/teleop_twist_keyboard)

2. https://www.youtube.com/watch?v=zIs6h3l5YgU

The video shows comparison of ORCA and Social forces model based collision avoidance. Other examples of different collision avoidance models for pedestrians can be found in http://gamma.cs.unc.edu/Menge/intro_vids.html

3. https://www.youtube.com/watch?v=Ue1hHk6KlGg

The video shows a use case of MengeROS where a robot (blue) in a simple office like environment with around (90) pedestrians learns a distribution of crowd over a grid using only local sensor observations. The left side shows the simulator, the right side shows the rviz visualization where the laser endpoints are in red. The dark regions of the grid indicate the likelihood of dense crowds.
