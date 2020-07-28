# RoboticsND_HomeServiceRobot
<<<<<<< HEAD
## Home Service Robot
A home service robot is a robot which will pick up and drop off an object per request. This ROS project simulates a TurtleBot picking up a virtual object and dropping off the virtual object in a specified drop off location.
## Localization
Localization is done using Adaptive Monte Carlo Localization (AMCL) ROS package. The default ROS parameters stated in turtlebot/turtlebot_simulator is used since it produced acceptable localization results.
## Mapping
Initially gmapping was used to attempt to make a map of the Gazebo environment however pgm_map_creator was chosen since gmapping yielded poor results. Gmapping is a laser based SLAM method typicaly yielding good data in practice. However, the Turtlebot LiDAR has a very small field of view and insufficient range such that SLAM took on the order of hours to generate a marginally sufficient map. Therefore, since pgm_map_creator will directly generate a ground truth map from Gazebo, the project was simplified. Future work can be done to implement a Hokuyo 3D LiDAR in a custom robot. Or an RGB-D camera could be used in conjunction with a Hokuyo LiDAR in order to use RTAB-Map algorithm for mapping.
=======
>>>>>>> df0556cdab71182d7149c04494693f4fc8cb0d2c
