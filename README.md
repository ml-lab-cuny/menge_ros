# menge_ros
Crowd simulation tool for robot navigation based on Menge(http://gamma.cs.unc.edu/Menge/) and ROS.
It extends Menge to introduce a robot agent into the crowd and which can be controller by external ROS compatible controllers. In other worlds now menge becomes a combined simulator for both crowd and robots.

Installation Guidelines: (Tested For Ubuntu 14.04 with ROS Indigo)

1. Download the software into the catkin workspace of ROS.
2. Use catkin\_make to recompile the ROS packages.
3. RUN the simulator using the command rosrun menge\_sim menge\_sim -p examples\core\tradeshow.xml
4. The previous command should start the simulator with tradeshow world with as many robots as specified in the examples\core\tradeshow\tradeshowS.xml
5. Press space to start the simulation
6. The robot in the simulator can be controlled externally using any controller than sends http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html messages

Examples:
1. https://youtu.be/Q-qEu4oBmsw

The video shows the tradeshow world in action with a single robot that is controlled by a external controller (http://wiki.ros.org/teleop_twist_keyboard)


2. 
