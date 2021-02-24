# ck9
Code for CK-9 Robotics Development Kit

To buy CK-9, click here: https://centaurirobotics.in/dev/buy.html

## Software required:
1. Ubuntu 16.04 or higher
2. ROS Kinetic Kame or higher

## Hardware required:
1. CK-9 Robotics Dev Kit (Includes Motors with encoders, LiDaR, precisely cut and sturdy modular base, wheels, castors)
2. 2 x Microcontroller. Example: Arduino Uno (One for IMU, one for base controller)
3. Robot Computer (Example: Raspberry Pi 3 Model B)
4. Battery
5. Workstation computer (PC with Linux Ubuntu 16.04 or higher)

## How to clone?
- ```cd catkin_ws/src```
- ```git clone https://github.com/centauri-robotics/ck9```
- ```cd ~/catkin_ws```
- ```catkin_make```
- ```source ~/catkin_ws/devel/setup.bash```

## How to teleop?
- ```roslaunch ck9_base minimal.launch```
- ```roslaunch turtlebot_teleop keyboard_teleop.launch```

That's it play around with your CK-9 and move it around with your PC keyboard!

## How to create a map/run SLAM?
- ```roslaunch ck9_base minimal.launch```
- ```roslaunch turtlebot_teleop keyboard_teleop.launch```
- ```roslaunch ck9_navigation slam.launch```
- ```roscd ck9/rviz``` and then ```rviz -d slam.rviz```
- View the bot and the map being created in rviz, and move around the bot with teleop to build a full map
- To save the map: ```rosrun map_server map_saver -f map_name``` with terminal at the location where you want to save the map
- You now have a map ready to run your bot autonomously on!

## How to run autonomous navigation?
- ```roslaunch ck9 minimal.launch```
- ```roslaunch ck9_navigation navigate.launch map_file:=/home/username/path_to_map.yaml``` 
- ```roscd ck9/rviz``` and then ```rviz -d navigate.rviz```
- Use the "2D Nav Goal" button in rviz to publish your navigation goals and see your CK-9 in action!
