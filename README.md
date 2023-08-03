# A Simple Pose Tracking Controller


https://raw.githubusercontent.com/weizhe-chen/tracking_pid/main/.github/images/tracking_demo.mp4

## Get Started

Install TurtleBot3 simulation following the [PC Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup) section and [Gazebo Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation) section.

```
mkdir ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/Weizhe-Chen/tracking_pid.git
cd ~/catkin_ws
catkin_make # or catkin build if you are using catkin tools
. devel/setup.bash
roslaunch tracking_pid demo.launch
```
