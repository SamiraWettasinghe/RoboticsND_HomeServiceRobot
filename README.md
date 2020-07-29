# RoboticsND_HomeServiceRobot
## Home Service Robot
A home service robot is a robot which will pick up and drop off an object per request. This ROS project simulates a TurtleBot picking up a virtual object and dropping off the virtual object in a specified drop off location.
## Mapping
Initially [gmapping](http://wiki.ros.org/gmapping) was used to attempt to make a map of the Gazebo environment however [pgm_map_creator](https://github.com/hyfan1116/pgm_map_creator) was chosen since gmapping yielded poor results. Gmapping is a laser based SLAM method typicaly yielding good data in practice. However, the Turtlebot LiDAR has a very small field of view and insufficient range such that SLAM took on the order of hours to generate a marginally sufficient map. Therefore, since pgm_map_creator will directly generate a ground truth map from Gazebo, the project was simplified. This map can be found below. Future work can be done to implement a Hokuyo 3D LiDAR in a custom robot. Or an RGB-D camera could be used in conjunction with a Hokuyo LiDAR to use [RTAB-Map](http://wiki.ros.org/rtabmap) algorithm for mapping.

![map](/images/map.png)
## Localization
Localization is done using [Adaptive Monte Carlo Localization (AMCL) ROS package](http://wiki.ros.org/amcl). The default ROS parameters stated in turtlebot/turtlebot_simulator is used since it produced acceptable localization results.
## Path Planning
The [ROS Navigation stack](http://wiki.ros.org/navigation) is used for path planning. This algorithm is based on Dijkstra's pathfinding algorithm which is a variation of Uniform Cost Search algorithm. The challenge is to determine a viable path given a previously mapped environment with appropriate inflation layers and cost map.

Given pick up location coordinates and pose, the task of path planning is to generate a viable trajectory to the object location. Once the object is picked up, drop off coordinates and pose is provided and path planning is to generate another viable trajectory towards the goal location.
## Full Simulation
To simulate the robot picking up objects and dropping off objects, [visualization_msgs/Marker](http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes) messages was used to send a blue square visualization to the pick up and drop off location. Once Turtlebot has navigated to the pick up location, the robot will wait 5 seconds to simulate the process of picking up an object. After this process is completed, Turtlebot will travel to the drop off location and once reached, a blue square will appear to simulate that the robot has successfully dropped off the object.

This process is done through the add_markers node and the pick_objects node. A service is defined in the pick_objects node which will take in a location for the navigation stack. The client is add_markers which will display blue squares and provide locations for Turtlebot to travel to.