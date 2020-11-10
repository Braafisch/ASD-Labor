# car_demo

A simple example for running a vehicle simulation in Gazebo. 
This repository has been forked from https://github.com/osrf/car_demo.

To run this
* install ros-desktop-full and gazebo
* install fake localization, map server, and smach tools:
    * `sudo apt update`
    * `sudo apt upgrade`
    * `sudo apt install ros-melodic-fake-localization ros-melodic-map-server`
    * `sudo apt install ros-melodic-smach ros-melodic-smach-ros ros-melodic-smach-msgs ros-melodic-smach-viewer python-gtk2` 
* create a workspace, e.g. `mkdir -p catkin_ws/src` 
* clone this repo into `catkin_ws/src`
* run `catkin_make` 
* run `source devel/setup.bash`
* start simulation with `roslaunch car_demo demo_keyboard.launch`
* from directory `catkin_ws/src/car_demo/nodes` run (in a separate terminal)
    * `./sample_node.py`

Some helpful vehicle parameters:
* L=2.7: [m] Wheel base of vehicle
* veh_dim_x, veh_dim_y= 4.645, 1.76]: [m] size of vehicle (length, width)

(Thao Dang, HS Esslingen)