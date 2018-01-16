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
catkin_create_pkg imu_pkg std_msgs rospy
cd ~/catkin_ws
catkin_make
. ~/catkin_ws/devel/setup.bash
```

Add ros_imu.py and display_3D_visualization.py to the package under imu_pkg/src/scripts (you may need to create the scripts folder). Edit the python scripts with the correct port name. Make the scripts executable:
```sh
chmod +x ~/catkin_ws/src/imu_pkg/scripts/ros_imu.py
```

**Viewing with ros_imu**  
With the IMU connected to the computer and running the firmware, run ros_imu.py. To see a text output, echo the topic "imu". To see a 3D visualization, run display_3D_visualization.py  
Run  ```roscore```
```sh
catkin_make
source devel/setup.bash
rosrun imu_pkg ros_imu.py
```
To view imu data, enter the following in a separate terminal while ros_imu is running:
```sh
rostopic echo imu
```
To see a 3D visualization, enter the following in a seperate terminal while ros_imu.py is running:
```
rosrun imu_pkg display_3D_visualization.py
```

## Troubleshooting
There is a known issue with the IMU in that the internal euler and qaternion calculator creates random spikes in the data. Using just the quaternions seems to produce the minimum amount of spikes. You can control which data the IMU calculates by sending serial commands "q" for quaternions and "e" for euler to toggle them on or off (you can do this in a serial moniter or by editing ros_imu.py with `ser.write('q'+chr(13))`). Code to calculate euler angles from quaternions is commented in ros_imu.py but it is not necessary for the other programs. Sparkfun may fix this issue at some point.

