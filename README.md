# ROS2 RACECAR foxy

full stack navigation system

## Description

overall its working it can navigate around the map. ros2 foxy expired and no longer supported also most new necessay packages doesnt support I am moving to ros2 humble. 

## Getting Started

### Dependencies // sudo apt get install

* UBUNTU 20.04 
* [ROS2 FOXY](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)     
* [ROS2 CONTROL AND CONTROLLERS](https://control.ros.org/foxy/index.html) ```ros-foxy-ros2-control, ros-foxy-ros2-controllers```
* [ROS2 SLAM_TOOLBOX](https://github.com/SteveMacenski/slam_toolbox) - ```ros-foxy-slam-toolbox```
* [ROS2 NAVIGATION](https://docs.nav2.org/getting_started/index.html) - ```ros-foxy-navigation2, ros-foxy-nav2-bringup```
* [ROS2 ROBOT_LOCALIZATION](http://docs.ros.org/en/melodic/api/robot_localization/html/index.html) - ```ros-foxy-robot-localization```
* [TWIST-MUX](http://wiki.ros.org/twist_mux) - ```ros-foxy-twist-mux```
### BUILDING
* locate the racecar-ROS2-foxy folder then build and add udev files
```
cd racecar-ROS2-foxy

colcon build

cd udev

bash init.sh
```


### Executing program

* launching motor, imu, lidar, robot-localization, twist-mux
```
ros2 launch racecar_bringup racecar.launch.py
```
* launching navigation stack
```
ros2 launch racecar_bringup nav2_bringup.py
```
* if you want to control the car on your phone or browser 

``` 
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

cd Joystick_ackerman
python3 -m http.server 8000 & # ipv4
python3 -m http.server 8000 --bind :: # ipv6
```


## Authors

 
gmail - dony_uzbguy@gmail.com

twitter - [@dony_morph](https://x.com/dony_morph)

## Other version

* ros1 noetic - https://github.com/JiaNing-Z/racecar
* ros1 kinetic - https://github.com/ART-Robot-Release/racecar
* ros1 melodic - https://github.com/ART-Robot-Release/racecar/tree/melodic