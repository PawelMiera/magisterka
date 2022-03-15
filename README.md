#### Quick tutorial to install and run ROS, Gazebo, PX4 on custom map with custom hardware (Depth camera, Lidar, Optical FLow, Height lidar and Light)

First install everything with one script from this tutorial (Ubuntu 18.04 is required):  
https://docs.px4.io/master/en/dev_setup/dev_env_linux_ubuntu.html#ros-gazebo

For Ubuntu 20.04 here is nice tutorial:  
https://www.youtube.com/watch?v=1FpJvUVPxL0

Next you need to download and build PX4:

```
(in known directory, can be home)
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
no_sim=1 make px4_sitl_default gazebo
```

Install catkin (ros builds packages with this tool) dependencies:

```
sudo apt-get install python-wstool python-rosinstall-generator python-catkin-tools
```

Then, initialize the catkin workspace, all the projects are located here:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
```

Install dependencies (mavros, mavlink):

```
sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras


OR harder way:

cd ~/catkin_ws
wstool init ~/catkin_ws/src

rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src --rosdistro `echo $ROS_DISTRO` -y

catkin build

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

```

Update global variables
```
source ~/.bashrc
```

Install geographiclib dependancy 
```
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
```

Now for everything to work fine you need te setup your .bashrc for ROS to see all needed packages.

```
sudo nano ~/.bashrc
```

Here is the end of my .bashrc you need to adjust it to your paths.

```
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
GAZEBO_MODEL_PATH=:/home/pawel/PX4-Autopilot/Tools/sitl_gazebo/models:/home/pawel/gazebo_models:/home/pawel/catkin_ws/src/magisterka/models
GAZEBO_PLUGIN_PATH=/home/pawel/PX4-Autopilot/build/px4_sitl_default/build_gazebo
LD_LIBRARY_PATH=/home/pawel/catkin_ws/devel/lib:/opt/ros/melodic/lib:/home/pawel/PX4-Autopilot/build/px4_sitl_default/build_gazebo
ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/pawel/PX4-Autopilot:/home/pawel/PX4-Autopilot/Tools/sitl_gazebo
```
```
ctrl + s 
ctrl + x
source ~/.bashrc
```

Now clone this repository into catkin workspace:
```
cd ~/catkin_ws/src
git clone https://github.com/PawelMiera/magisterka.git
```
And build it:
```
catkin build
```

To run the simulation:
```
roslaunch magisterka magisterka.launch
```

Use rviz to display all sensor data (the configuration file is named default.rviz you can open it or replace the default file in /opt/ros/melodic/share/rviz):
```
rviz
```

The most important files are:  
magisterka.launch  
CMakeLists.txt  
cave_world.world  
models directory (you can change sensor settings in sdf files)  

If you want to use another map you need to add drone model to your world file:

```
<model name="agh_iris">
  <pose> 10 -20.5 0.1 0 0 3.14</pose>

  <include>
    <uri>model://iris</uri> 
  </include>
  
  <include>
    <uri>model://agh_depth_camera</uri>
    <pose>0.1 0 0 0 0 0</pose>
  </include>

  <include>
    <uri>model://agh_lidar</uri>
    <pose>0.1 0 0 0 0 0</pose>
  </include>

  <include>
    <uri>model://agh_height_lidar</uri>
    <pose>0 0 -0.05 0 0 0</pose>
  </include>

  <include>
    <uri>model://agh_opflow</uri>
    <pose>0.05 0 -0.05 0 0 0</pose>
  </include>

  <include>
    <uri>model://agh_light</uri>
    <pose>0 0 0 0 0 0</pose>
  </include>

  <joint name="depth_camera_joint" type="fixed">
    <child>agh_depth_camera::link</child>
    <parent>iris::base_link</parent>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
 
  <joint name="hokuyo_joint" type="fixed">
    <pose>0 0 0 0 0 0</pose>
    <parent>iris::base_link</parent>
    <child>agh_lidar::link</child>
  </joint>

  <joint name="opflow_joint" type="revolute">
    <parent>iris::base_link</parent>
    <child>agh_opflow::link</child>
    <axis>
      <xyz>0 0 1</xyz>
      <limit>
        <upper>0</upper>
        <lower>0</lower>
      </limit>
    </axis>
  </joint>

  <joint name="lidar_joint" type="fixed">
    <parent>iris::base_link</parent>
    <child>agh_height_lidar::link</child>
  </joint>

  <joint name="light_joint" type="fixed">
    <child>agh_light::link</child>
    <parent>iris::base_link</parent>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

</model>
```

You can start the flight with QGroundControl.



