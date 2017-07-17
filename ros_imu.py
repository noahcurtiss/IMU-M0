#!/usr/bin/env python

import rospy
import serial
import string
import math
import sys

from sensor_msgs.msg import Imu
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

rospy.init_node("imu_node")
#We only care about the most recent measurement, i.e. queue_size=1
pub = rospy.Publisher('imu', Imu, queue_size=1)
diag_pub = rospy.Publisher('diagnostics', DiagnosticArray, queue_size=1)
diag_pub_time = rospy.get_time();

accel_factor = 9.806  #Imu accelerometer output is in g so we must convert to m/s^2
degrees2rad = math.pi/180
seq = 0

imuMsg = Imu()

rate = rospy.Rate(10)

#covariance has not yet been determined. Default is 0
# imuMsg.orientation_covariance = [
# 0 , 0 , 0,
# 0, 0, 0,
# 0, 0, 0
# ]
# imuMsg.angular_velocity_covariance = [
# 0, 0 , 0,
# 0 , 0., 0,
# 0 , 0 , 0
# ]
# imuMsg.linear_acceleration_covariance = [
# 0 , 0 , 0,
# 0 , 0, 0,
# 0 , 0 , 0
# ]

default_port='/dev/ttyACM1' #replace with port name
port = rospy.get_param('~port', default_port)

rospy.loginfo("Opening %s...", port)
try:
    ser = serial.Serial(port=port, baudrate=115200, timeout=1)
except serial.serialutil.SerialException:
    rospy.logerr("IMU not found at port "+port + ". Did you specify the correct port in the launch file?")
    sys.exit(0)

temp_line = ser.readline()
temp_words = string.split(temp_line,", ")
# if len(temp_words)<15:
#     ser.write('qe'+chr(13)) #includes quaternion and euler angles in serial data

rospy.loginfo("Flushing first 200 IMU entries...")
for x in range(0, 200):
    line = ser.readline()
rospy.loginfo("Publishing IMU data...")


while not rospy.is_shutdown(): 
    line = ser.readline()              # read serial data
    words = string.split(line,", ")    # Fields split
    
    #yaw, pitch and roll in rad
#     yaw = float(words[16])*degrees2rad
#     pitch = float(words[14])*degrees2rad
#     roll = float(words[15])*degrees2rad

    #linear acceleration in m/s^2
    imuMsg.linear_acceleration.x = float(words[1]) * accel_factor
    imuMsg.linear_acceleration.y = float(words[2]) * accel_factor
    imuMsg.linear_acceleration.z = float(words[3]) * accel_factor

    #angular velocity in rad/s
    imuMsg.angular_velocity.x = float(words[4]) * degrees2rad
    imuMsg.angular_velocity.y = float(words[5]) * degrees2rad
    imuMsg.angular_velocity.z = float(words[6]) * degrees2rad

    #quaternions
#     imuMsg.orientation.x = float(words[11])
#     imuMsg.orientation.y = float(words[12])
#     imuMsg.orientation.z = float(words[13])
#     imuMsg.orientation.w = float(words[10])
    
    imuMsg.header.stamp= rospy.Time.now()
    imuMsg.header.frame_id = 'base_imu_link'
    imuMsg.header.seq = seq
    seq = seq + 1
    pub.publish(imuMsg)
    
    
    if (diag_pub_time < rospy.get_time()) :
        diag_pub_time += 1
        diag_arr = DiagnosticArray()
        diag_arr.header.stamp = rospy.get_rostime()
        diag_arr.header.frame_id = '1'
        diag_msg = DiagnosticStatus()
        diag_msg.name = 'Razor_Imu'
        diag_msg.level = DiagnosticStatus.OK
        diag_msg.message = 'Received AHRS measurement'
#         diag_msg.values.append(KeyValue('roll (deg)',
#                                 str(roll*(180.0/math.pi))))
#         diag_msg.values.append(KeyValue('pitch (deg)',
#                                 str(pitch*(180.0/math.pi))))
#         diag_msg.values.append(KeyValue('yaw (deg)',
#                                 str(yaw*(180.0/math.pi))))
        diag_msg.values.append(KeyValue('sequence number', str(seq)))
        diag_arr.status.append(diag_msg)
        diag_pub.publish(diag_arr)
        
ser.close
#f.close
