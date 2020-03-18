ROS/Gazebo setup to perform multi-robot behavior based robotics testing.
Created with Ron Arkin's Motor Schema approach in mind. 
Gazebo simulation environment that runs three ClearPath Husky bots, each controlled with ther own instance of RVIZ.  
Each Husky uses Vector Field Histogram local path planning and, in addition to the default Move Base inflation layer and obstacle_lasers layer, has three additional costmap layers available: 
-simple_layer
-grid_layer
-second_layer
The layers currently don't do anything, but users can modify them to the desired behavior and create a combined cost map. 
To run, build the code and run the following commands:
-roslaunch husky_gazebo multi_husky_simple_bbc.launch
-roslaunch husky_viz view_bbc.launch
-roslaunch husky_navigation move_base_mapless_bbc.launch

Developed running Ubuntu 16.04 LTS on ROS Kinetic. 

Original ClearPath Husky code base was from https://github.com/husky/husky. Has been modified.
Original Vector Field Historgram implementation was from https://github.com/AugustoPB/vfh_local_planner. Has been modified.
Code to create new costmaps was based on tutorials found at http://wiki.ros.org/costmap_2d/Tutorials/Creating%20a%20New%20Layer


