# ORB-SLAM2 on ROS with DJI Tello
This repository provides an ORB-SLAM2 example on Robot Operating System (ROS) with DJI Tello drone.
DJI Tello sends video stream to the ORB-SLAM2, and it publishes 3-DoF position and 3-DoF orientation of the DJI Tello.
GUI will allow you to control the DJI Tello and command it to move along the x, y, z, roll, pitch, and yaw directions.
You can also find a joystick/keyboard to control the DJI Tello instead of using your DJI Tello app.

![Image of GUI](https://raw.githubusercontent.com/tau-adl/Tello_ROS_ORBSLAM/master/Images/tello_ui.png)


# 1. Installation Guide

* 1) Environments: Ubuntu 18.04.5 LTS (64-bit) + ROS Melodic + MSI GS60 (GeForce GTX 970M)

* 2) Robot Operating System (ROS) first by following this page: http://wiki.ros.org/melodic/Installation/Ubuntu

* 3) Install some packages.
```
sudo apt-get install libeigen3-dev ffmpeg
sudo apt-get install python-catkin-tools
sudo apt-get install ros-melodic-joystick-drivers
sudo apt-get install python-imaging-tk
sudo apt-get install python-pip
```

* 4) Pangolin for ORB-SLAM2 visualization: https://github.com/stevenlovegrove/Pangolin
```
git clone https://github.com/stevenlovegrove/Pangolin.git
sudo apt-get install libgl1-mesa-dev libglew-dev libxkbcommon-dev
cd Pangolin
mkdir build
cd build
cmake ..
cmake --build .
```

* 5) CMake latest version at least > 3.14 for H264 Decoder: https://askubuntu.com/questions/355565/how-do-i-install-the-latest-version-of-cmake-from-the-command-line

* 6) H264 Decoder for DJI Tello video stream: https://github.com/DaWelter/h264decoder
```
git clone https://github.com/DaWelter/h264decoder.git
sudo apt install libswscale-dev libavcodec-dev libavutil-dev
cd h264decoder
pip install .

mkdir build
cd build
cmake ..
make
```





now copy it to python path
```
sudo cp ~/ROS/h264decoder/libh264decoder.so /usr/local/lib/python2.7/dist-packages
```










## Installing our version of TelloPy
based on https://github.com/dji-sdk/Tello-Python and https://github.com/hanyazou/TelloPy
```
cd ~/Documents/Tello_ROS_ORBSLAM/TelloPy
sudo python setup.py install
```


## Installing dependencies for ROS
```
cd ~/Documents/Tello_ROS_ORBSLAM/ROS/tello_catkin_ws/
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```


## Build the code:
```
cd ~/Documents/Tello_ROS_ORBSLAM/ROS/tello_catkin_ws/
catkin init
catkin clean
catkin build -j2
```





If it doesn’t work, make sure you changed the makefile to the wanted version of ROS
## Add the enviroment setup to bashrc
```
echo "source $PWD/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
# Installing ccm_slam
based on https://github.com/VIS4ROB-lab/ccm_slam
## Compile DBoW2:
```
cd ~/ROS/Tello_ROS_ORBSLAM/ROS/ccmslam_ws/src/ccm_slam/cslam/thirdparty/DBoW2/
mkdir build
cd build
cmake ..
make -j8
```
## Compile g2o:
```
cd ~/ROS/Tello_ROS_ORBSLAM/ROS/ccmslam_ws/src/ccm_slam/cslam/thirdparty/g2o
mkdir build
cd build
cmake --cmake-args -DG2O_U14=0 ..
make -j8
```
## Unzip Vocabulary:
```
cd ~/ROS/Tello_ROS_ORBSLAM/ROS/ccmslam_ws/src/ccm_slam/cslam/conf
unzip ORBvoc.txt.zip
```
## Build the code:
```
cd ~/ROS/Tello_ROS_ORBSLAM/ROS/ccmslam_ws/
source /opt/ros/melodic/setup.bash
catkin init
catkin config --extend /opt/ros/melodic
catkin build ccmslam --cmake-args -DG2O_U14=0 -DCMAKE_BUILD_TYPE=Release
```

If Gives error -  ROS distro neither indigo nor kinetic - change the makefile, use CmakeFile_changed2.
## Add the enviroment setup to bashrc
```
echo "source $PWD/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Notes
### Flock
we have used code from https://github.com/clydemcqueen/flock



















# 2.



# 3.





##### Calibrate Z!
Pressing this button will make the Tello elevate ~0.5 meters and then descent ~0.5 meters.

During this time the tello's internal height sensor will be sampled among with the height that is being published by the SLAM algorithm.

At the end of the movement the control calculates:

<img src="https://render.githubusercontent.com/render/math?math=\text{real_world_scale}=\frac{ \Delta \text{height_sensor}} {\Delta \text{SLAM_height}}">

The control will be using that factor to go from the SLAM coordinate system to the real world coordinate system.

##### Publish Command!
Pushing this button will send the coordinates in the boxes (x, y, z, yaw) to the control, and the control will navigate the tello to those coordinates.
###### note that the control will control the Tello only if the "Toggle Slam Control!" Button is pressed!

##### Stay In Place!
This button will command the Tello to stay in the current place using the control.

#### Real World Position Section
![Real World](https://raw.githubusercontent.com/tau-adl/Tello_ROS_ORBSLAM/master/Images/tello_ui_real_world_position.png)

In this section we can observe the tello's current coordinates in the real world coordinates.
The Altitude Scale is the factor that was calculated during the Calibrate Z! process.

#### Rotated SLAM Coordinates Section
![Command Position](https://raw.githubusercontent.com/tau-adl/Tello_ROS_ORBSLAM/master/Images/tello_ui_rotated_slam.png)

In this section we can observe the tello's current coordinates in the SLAM coordinates (after rotating the coordinate system to compensate for the angle of the camera of the Tello).

#### Delta Between Command and Real World Section
![Delta](https://raw.githubusercontent.com/tau-adl/Tello_ROS_ORBSLAM/master/Images/tello_ui_delta.png)

In this section we can observe the difference between the Tello's current pose, to the desired pose we commanded.

#### Speed Section
![Speed](https://raw.githubusercontent.com/tau-adl/Tello_ROS_ORBSLAM/master/Images/tello_ui_speed.png)

In this section we can observe the current speed in all the axes (pitch, roll, throttle, yaw) of the Tello.
Also, we can see some side information about the Tello, likt the Altitude and the remaining Battery[%].
##### Toggle Slam Control!
This is a protection button. Pressing this button will allow the control to take over the Tello's speed control.

#### Manual Control Section
![Manual Control](https://raw.githubusercontent.com/tau-adl/Tello_ROS_ORBSLAM/master/Images/tello_ui_manual_control.png)

In this section we can manually control the speeds of the Tello in all axes.
Just insert the wanted speed (0-1) in each axis and press Manual Control Set!
To stop press the Manual Control Clear! button.

#### Trajectory Control Section
![Trajectory](https://raw.githubusercontent.com/tau-adl/Tello_ROS_ORBSLAM/master/Images/tello_ui_trajectory.png)

This section allows to command the Tello to go to many points one after the other.
You can either insert the coordinates manually in the boxes, or load the using Load File button.
Eventually press the Publish Trajectory! button to start the trajectory.
###### note that the control will control the Tello only if the "Toggle Slam Control!" Button is pressed!

#### Toggle Use Merged Coordinates
This button is used in the ccmslam algorithm.
After the two drones have merged a map, our modification to ccmslam algorithm allows to control the drones using the same coordinate system.
In other words, after the map was merged, pressing this button will make the controller of each drone to use the same coordinate system.

## Tello Viewers:
In the Viewer we can observe the video stream received from the SLAM algorithm, among with the cloud map in the X-Y plane.
### Tello Client 0:
![Image of Tello Client 0](https://raw.githubusercontent.com/tau-adl/Tello_ROS_ORBSLAM/master/Images/tello_client0.png)


### Tello Client 1:
![Image of Tello Client 1](https://raw.githubusercontent.com/tau-adl/Tello_ROS_ORBSLAM/master/Images/tello_client1.png)

# Usage
## orbslam2
```
roslaunch flock_driver orbslam2_with_cloud_map.launch
```
## ccmslam
### Server:
```
roslaunch ccmslam tello_Server.launch
```
### Client0:
```
roslaunch ccmslam tello_Client0.launch
```
### Client1:
```
roslaunch ccmslam tello_Client1.launch
```
