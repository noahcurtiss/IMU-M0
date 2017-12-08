# IMU-M0

## Uploading Firmware in Arduino
Add 9DoF_Razor_M0_Firmware.ino and config.h to your arduino library. Upload them to the imu through a micro USB cable. Additional setup instructions can be found on the [9dof Razor IMU M0 Hookup Guide](https://learn.sparkfun.com/tutorials/9dof-razor-imu-m0-hookup-guide). It may be necessary to reset the IMU before code can be uploaded.

To do this, connect a header cable to a GND pin and touch it to the reset pin twice. Wait about 20 seconds and the IMU should be ready. The reset pin can be found in the group of small pins next to the I2C header. It is the furthest pin in that group from the GND pin.

If you get "error=13", you may need permission to access certain files which can be fixed using
```sh
sudo chmod -R 755 FILENAME
```

## Viewing with ROS
Create, build, and source a ROS new package dependent on rospy and std_msgs:  

**Create Catkin Workspace**
```sh
source /opt/ros/kinetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```
**Create ROS Package**
```sh
cd ~/catkin_ws/src
catkin_create_pkg imu std_msgs rospy
cd ~/catkin_ws
catkin_make
. ~/catkin_ws/devel/setup.bash
```

Add ros_imu.py and display_3D_visualization.py to the package. Run catkin_make and source your workspace's setup file. Edit the python scripts with the correct port name. With the IMU connected to the computer and running the firmware, run ros_imu.py. To see a text output, echo the topic "imu". To see a 3D visualization, run display_3D_visualization.py.

## Troubleshooting
There is a known issue with the IMU in that the internal euler and qaternion calculator creates random spikes in the data. To avoid this, be sure that these calculations are disabled by checking the serial moniter and sending the command "qe" to toggle them on or off. If SparkFun fixes the issue, or if you don't care about the data spikes, there is commented code in ros_imu.py that reads the quaternions and euler angles from the IMU. Until then, a method of calculating these values within the python script will need to be added. Without these values, display_3D_visualization.py, and many other processes, will not run.

