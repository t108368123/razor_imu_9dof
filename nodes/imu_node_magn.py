#!/usr/bin/env python

# Copyright (c) 2012, Tang Tiong Yew
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
import serial
import string
import math
import sys

#from time import time
from sensor_msgs.msg import MagneticField
from tf.transformations import quaternion_from_euler
from dynamic_reconfigure.server import Server
from razor_imu_9dof.cfg import imuConfig
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

degrees2rad = math.pi/180.0
imu_yaw_calibration = 0.0

# Callback for dynamic reconfigure requests
def reconfig_callback(config, level):
    global imu_yaw_calibration
    rospy.loginfo("""Reconfigure request for yaw_calibration: %d""" %(config['yaw_calibration']))
    #if imu_yaw_calibration != config('yaw_calibration'):
    imu_yaw_calibration = config['yaw_calibration']
    rospy.loginfo("Set imu_yaw_calibration to %d" % (imu_yaw_calibration))
    return config

rospy.init_node("razor_node")

imuMsg = MagneticField()

imuMsg.magnetic_field_covariance = [
0 , 0 , 0,
0 , 0 , 0,
0 , 0 , 0
]

# read basic information
port = rospy.get_param('~port', '/dev/ttyUSB0')
topic = rospy.get_param('~topic', 'imu_magn')
frame_id = rospy.get_param('~frame_id', 'base_imu_link')

# read calibration parameters

# accelerometer
accel_x_min = rospy.get_param('~accel_x_min', -250.0)
accel_x_max = rospy.get_param('~accel_x_max', 250.0)
accel_y_min = rospy.get_param('~accel_y_min', -250.0)
accel_y_max = rospy.get_param('~accel_y_max', 250.0)
accel_z_min = rospy.get_param('~accel_z_min', -250.0)
accel_z_max = rospy.get_param('~accel_z_max', 250.0)

# magnetometer
magn_x_min = rospy.get_param('~magn_x_min', -600.0)
magn_x_max = rospy.get_param('~magn_x_max', 600.0)
magn_y_min = rospy.get_param('~magn_y_min', -600.0)
magn_y_max = rospy.get_param('~magn_y_max', 600.0)
magn_z_min = rospy.get_param('~magn_z_min', -600.0)
magn_z_max = rospy.get_param('~magn_z_max', 600.0)
calibration_magn_use_extended = rospy.get_param('~calibration_magn_use_extended', False)
magn_ellipsoid_center = rospy.get_param('~magn_ellipsoid_center', [0, 0, 0])
magn_ellipsoid_transform = rospy.get_param('~magn_ellipsoid_transform', [[0, 0, 0], [0, 0, 0], [0, 0, 0]])
imu_yaw_calibration = rospy.get_param('~imu_yaw_calibration', 0.0)

# gyroscope
gyro_average_offset_x = rospy.get_param('~gyro_average_offset_x', 0.0)
gyro_average_offset_y = rospy.get_param('~gyro_average_offset_y', 0.0)
gyro_average_offset_z = rospy.get_param('~gyro_average_offset_z', 0.0)

#rospy.loginfo("%f %f %f %f %f %f", accel_x_min, accel_x_max, accel_y_min, accel_y_max, accel_z_min, accel_z_max)
#rospy.loginfo("%f %f %f %f %f %f", magn_x_min, magn_x_max, magn_y_min, magn_y_max, magn_z_min, magn_z_max)
#rospy.loginfo("%s %s %s", str(calibration_magn_use_extended), str(magn_ellipsoid_center), str(magn_ellipsoid_transform[0][0]))
#rospy.loginfo("%f %f %f", gyro_average_offset_x, gyro_average_offset_y, gyro_average_offset_z)

pub = rospy.Publisher(topic, MagneticField, queue_size=1)
srv = Server(imuConfig, reconfig_callback)  # define dynamic_reconfigure callback

# Check your COM port and baud rate
rospy.loginfo("Opening %s...", port)
try:
    ser = serial.Serial(port=port, baudrate=57600, timeout=1)
    #ser = serial.Serial(port=port, baudrate=57600, timeout=1, rtscts=True, dsrdtr=True) # For compatibility with some virtual serial ports (e.g. created by socat) in Python 2.7
except serial.serialutil.SerialException:
    rospy.logerr("IMU not found at port "+port + ". Did you specify the correct port in the launch file?")
    #exit
    sys.exit(2)

roll=0
pitch=0
yaw=0
seq=0
accel_factor = 9.806 / 256.0    # sensor reports accel as 256.0 = 1G (9.8m/s^2). Convert to m/s^2.
rospy.loginfo("Giving the razor IMU board 5 seconds to boot...")
rospy.sleep(5) # Sleep for 5 seconds to wait for the board to boot

### configure board ###
#stop datastream
ser.write(('#o0').encode("utf-8"))

#discard old input
#automatic flush - NOT WORKING
#ser.flushInput()  #discard old input, still in invalid format
#flush manually, as above command is not working
discard = ser.readlines() 

#set output mode
ser.write(('#ox').encode("utf-8")) # To start display angle and sensor reading in text

rospy.loginfo("Writing calibration values to razor IMU board...")
#set calibration values
ser.write(('#caxm' + str(accel_x_min)).encode("utf-8"))
ser.write(('#caxM' + str(accel_x_max)).encode("utf-8"))
ser.write(('#caym' + str(accel_y_min)).encode("utf-8"))
ser.write(('#cayM' + str(accel_y_max)).encode("utf-8"))
ser.write(('#cazm' + str(accel_z_min)).encode("utf-8"))
ser.write(('#cazM' + str(accel_z_max)).encode("utf-8"))

if (not calibration_magn_use_extended):
    ser.write(('#cmxm' + str(magn_x_min)).encode("utf-8"))
    ser.write(('#cmxM' + str(magn_x_max)).encode("utf-8"))
    ser.write(('#cmym' + str(magn_y_min)).encode("utf-8"))
    ser.write(('#cmyM' + str(magn_y_max)).encode("utf-8"))
    ser.write(('#cmzm' + str(magn_z_min)).encode("utf-8"))
    ser.write(('#cmzM' + str(magn_z_max)).encode("utf-8"))
else:
    ser.write(('#ccx' + str(magn_ellipsoid_center[0])).encode("utf-8"))
    ser.write(('#ccy' + str(magn_ellipsoid_center[1])).encode("utf-8"))
    ser.write(('#ccz' + str(magn_ellipsoid_center[2])).encode("utf-8"))
    ser.write(('#ctxX' + str(magn_ellipsoid_transform[0][0])).encode("utf-8"))
    ser.write(('#ctxY' + str(magn_ellipsoid_transform[0][1])).encode("utf-8"))
    ser.write(('#ctxZ' + str(magn_ellipsoid_transform[0][2])).encode("utf-8"))
    ser.write(('#ctyX' + str(magn_ellipsoid_transform[1][0])).encode("utf-8"))
    ser.write(('#ctyY' + str(magn_ellipsoid_transform[1][1])).encode("utf-8"))
    ser.write(('#ctyZ' + str(magn_ellipsoid_transform[1][2])).encode("utf-8"))
    ser.write(('#ctzX' + str(magn_ellipsoid_transform[2][0])).encode("utf-8"))
    ser.write(('#ctzY' + str(magn_ellipsoid_transform[2][1])).encode("utf-8"))
    ser.write(('#ctzZ' + str(magn_ellipsoid_transform[2][2])).encode("utf-8"))

ser.write(('#cgx' + str(gyro_average_offset_x)).encode("utf-8"))
ser.write(('#cgy' + str(gyro_average_offset_y)).encode("utf-8"))
ser.write(('#cgz' + str(gyro_average_offset_z)).encode("utf-8"))

#print calibration values for verification by user
ser.flushInput()
ser.write(('#p').encode("utf-8"))
calib_data = ser.readlines()
calib_data_print = "Printing set calibration values:\r\n"
for row in calib_data:
    line = bytearray(row).decode("utf-8")
    calib_data_print += line
rospy.loginfo(calib_data_print)

#start datastream
ser.write(('#o1').encode("utf-8"))

#Output RAW SENSOR data
ser.write(('#osrt').encode("utf-8"))

#automatic flush - NOT WORKING
#ser.flushInput()  #discard old input, still in invalid format
#flush manually, as above command is not working - it breaks the serial connection
rospy.loginfo("Flushing first 200 IMU entries...")
for x in range(0, 200):
    line = bytearray(ser.readline()).decode("utf-8")
rospy.loginfo("Publishing IMU data...")
#f = open("raw_imu_data.log", 'w')

errcount = 0
while not rospy.is_shutdown():
    if (errcount > 10):
        break
    line = bytearray(ser.readline()).decode("utf-8")
    if (((line.find("#G-R=") == 1) or (line.find("#A-R=") == 1) or (line.find("#M-R=") == 1) == -1) or (line.find("\r\n") == -1)): 
        rospy.logwarn("Bad IMU data or bad sync")
        errcount = errcount+1
        continue
    if (line.find("#M-R=") == -1):
        continue
    line = line.replace("#M-R=","")   # Delete "#M-R="
    #f.write(line)                     # Write to the output log file
    line = line.replace("\r\n","")   # Delete "\r\n"
    words = line.split(",")    # Fields split
    if len(words) != 3:
        rospy.logwarn("Bad IMU data or bad sync")
        errcount = errcount+1
        continue
    else:
        errcount = 0
        imuMsg.magnetic_field.x = float(words[0])
        imuMsg.magnetic_field.y = float(words[1])
        imuMsg.magnetic_field.z = float(words[2])

    imuMsg.header.stamp= rospy.Time.now()
    imuMsg.header.frame_id = frame_id
    imuMsg.header.seq = seq
    seq = seq + 1
    pub.publish(imuMsg)
        
ser.close

#f.close

if (errcount > 10):
    sys.exit(10)
