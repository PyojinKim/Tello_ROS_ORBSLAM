# ORB-SLAM2 on ROS with DJI Tello
This repository provides an ORB-SLAM2 example on Robot Operating System (ROS) with DJI Tello drone.
DJI Tello sends video stream to the ORB-SLAM2, and it publishes 3-DoF position and 3-DoF orientation of the DJI Tello.
Control panel (GUI) will allow you to control the DJI Tello and command it to move along the x, y, z, roll, pitch, and yaw directions.
You can also find a joystick/keyboard to control the DJI Tello instead of using your DJI Tello app.

![Overview](https://github.com/PyojinKim/Tello_ROS_ORBSLAM/blob/master/images/overview.png)


# 1. Installation Guide
(1) Tested Environments: Ubuntu 18.04.5 LTS (64-bit) + ROS Melodic + MSI GS60 (GeForce GTX 970M)

(2) Install Robot Operating System (ROS) first following this page: http://wiki.ros.org/melodic/Installation/Ubuntu

(3) Eigen3, ffmpeg, and other trivial packages
```
sudo apt-get install libeigen3-dev ffmpeg
sudo apt-get install python-catkin-tools
sudo apt-get install ros-melodic-joystick-drivers
sudo apt-get install python-imaging-tk
sudo apt-get install python-pip
```

(4) Pangolin for ORB-SLAM2 visualization: https://github.com/stevenlovegrove/Pangolin
```
git clone https://github.com/stevenlovegrove/Pangolin.git
sudo apt-get install libgl1-mesa-dev libglew-dev libxkbcommon-dev
cd Pangolin
mkdir build
cd build
cmake ..
cmake --build .
```

(5) CMake latest version at least > 3.14 for H264 Decoder: [link1](https://askubuntu.com/questions/355565/how-do-i-install-the-latest-version-of-cmake-from-the-command-line)

(6) H264 Decoder for DJI Tello video stream: https://github.com/DaWelter/h264decoder
```
git clone https://github.com/DaWelter/h264decoder.git
sudo apt-get install libswscale-dev libavcodec-dev libavutil-dev
cd h264decoder
pip install .

mkdir build
cd build
cmake ..
make
```

(7) TelloPy and H264 Decoder from this repository
```
git clone https://github.com/PyojinKim/Tello_ROS_ORBSLAM.git
cd ~/Documents/Tello_ROS_ORBSLAM/TelloPy
sudo python setup.py install

cd ~/Documents/Tello_ROS_ORBSLAM/h264decoder
sudo apt-get install libswscale-dev libavcodec-dev libavutil-dev
sudo cp ~/Documents/Tello_ROS_ORBSLAM/h264decoder/Linux/libh264decoder.so /usr/local/lib/python2.7/dist-packages
```

(8) DJI Tello catkin workspace from this repository
```
cd ~/Documents/Tello_ROS_ORBSLAM/ROS/tello_catkin_ws/
rosdep update
rosdep install --from-paths src --ignore-src -r -y

catkin init
catkin clean
catkin build --mem-limit 70% -j1

echo "source $PWD/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```


# 2. Usage
* Connect DJI Tello through WiFi network such as TELLO-5B8FEF

* Run roslaunch like below command in the terminal. Enjoy! :)
```
roslaunch flock_driver orbslam2_with_cloud_map.launch
```






# 3. DJI Tello Control Panel (GUI)

![Overview](https://github.com/PyojinKim/Tello_ROS_ORBSLAM/blob/master/images/overview.png)

* `Calibrate Z!`: will make the DJI Tello elevate about 0.5 meters and then descend 0.5 meters.
With this movement, the internal height sensor of the DJI Tello will be used together with the height which is measured by the ORB-SLAM2.
The controller will compute and use this scale factor from the SLAM coordinates to the real-world coordinates.






##### Publish Command!
Pushing this button will send the coordinates in the boxes (x, y, z, yaw) to the control, and the control will navigate the tello to those coordinates.
###### note that the control will control the Tello only if the "Toggle Slam Control!" Button is pressed!






##### Toggle Slam Control!
This is a protection button. Pressing this button will allow the control to take over the Tello's speed control.

#### Manual Control Section
In this section we can manually control the speeds of the Tello in all axes.
Just insert the wanted speed (0-1) in each axis and press Manual Control Set!
To stop press the Manual Control Clear! button.




# 4. References
