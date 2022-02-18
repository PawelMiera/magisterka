# magisterka

#### Quick tutorial to install and run ROS, Gazebo, PX4 on custom map with custom hardware (depth camera, Lidar and Light)

First install everything with one script from this tutorial (Ubuntu 18.04 is required):  
https://docs.px4.io/master/en/dev_setup/dev_env_linux_ubuntu.html#ros-gazebo

Next you need to download and build PX4:

```
(in known directory, can be home)
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
no_sim=1 make px4_sitl_default gazebo

```
