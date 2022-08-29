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
from sensor_msgs.msg import Imu
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

imuMsg = Imu()
magMsg = MagneticField()

magMsg.magnetic_field_covariance = [
0 , 0 , 0,
0 , 0 , 0,
0 , 0 , 0
]

# Orientation covariance estimation:
# Observed orientation noise: 0.3 degrees in x, y, 0.6 degrees in z
# Magnetometer linearity: 0.1% of full scale (+/- 2 gauss) => 4 milligauss
# Earth's magnetic field strength is ~0.5 gauss, so magnetometer nonlinearity could
# cause ~0.8% yaw error (4mgauss/0.5 gauss = 0.008) => 2.8 degrees, or 0.050 radians
# i.e. variance in yaw: 0.0025
# Accelerometer non-linearity: 0.2% of 4G => 0.008G. This could cause
# static roll/pitch error of 0.8%, owing to gravity orientation sensing
# error => 2.8 degrees, or 0.05 radians. i.e. variance in roll/pitch: 0.0025
# so set all covariances the same.
imuMsg.orientation_covariance = [
0.0025 , 0 , 0,
0, 0.0025, 0,
0, 0, 0.0025
]

# Angular velocity covariance estimation:
# Observed gyro noise: 4 counts => 0.28 degrees/sec
# nonlinearity spec: 0.2% of full scale => 8 degrees/sec = 0.14 rad/sec
# Choosing the larger (0.14) as std dev, variance = 0.14^2 ~= 0.02
imuMsg.angular_velocity_covariance = [
0.02, 0 , 0,
0 , 0.02, 0,
0 , 0 , 0.02
]

# linear acceleration covariance estimation:
# observed acceleration noise: 5 counts => 20milli-G's ~= 0.2m/s^2
# nonliniarity spec: 0.5% of full scale => 0.2m/s^2
# Choosing 0.2 as std dev, variance = 0.2^2 = 0.04
imuMsg.linear_acceleration_covariance = [
0.04 , 0 , 0,
0 , 0.04, 0,
0 , 0 , 0.04
]

# read basic information
port = rospy.get_param('~port', '/dev/ttyUSB0')
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

# calibration param
acc_offset_x = 512/(accel_x_max-accel_x_min)
acc_cali_x= 256-(accel_x_max*acc_offset_x)
acc_offset_y = 512/(accel_y_max-accel_y_min)
acc_cali_y= 256-(accel_y_max*acc_offset_y)
acc_offset_z = 512/(accel_z_max-accel_z_min)
acc_cali_z= 256-(accel_z_max*acc_offset_z)

mag_offset_x = 1200/(magn_x_max-magn_x_min)
mag_cali_x= 600-(magn_x_max*mag_offset_x)
mag_offset_y = 1200/(magn_y_max-magn_y_min)
mag_cali_y= 600-(magn_y_max*mag_offset_y)
mag_offset_z = 1200/(magn_z_max-magn_z_min)
mag_cali_z= 600-(magn_z_max*mag_offset_z)

print("mox:",mag_offset_x)
print("moy:",mag_offset_y)
print("moz:",mag_offset_z)
print("mx:",mag_cali_x)
print("my:",mag_cali_y)
print("mz:",mag_cali_z)

pub_mag = rospy.Publisher("imu/magn", MagneticField, queue_size=1)
pub_imu = rospy.Publisher("imu/data_raw", Imu, queue_size=1)
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
seq_imu=0
seq_mag=0
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

acc_ok = 0
ang_ok = 0
errcount = 0
while not rospy.is_shutdown():
    if (errcount > 10):
        break
    line = bytearray(ser.readline()).decode("utf-8")
    if (((line.find("#G-R=") == 1) or (line.find("#A-R=") == 1) or (line.find("#M-R=") == 1) == -1) or (line.find("\r\n") == -1)): 
        rospy.logwarn("Bad IMU data or bad sync")
        errcount = errcount+1
        continue
    if (line.find("#M-R=") != -1):
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

            magMsg.magnetic_field.x = -(-(float(words[0])*mag_offset_x - mag_cali_x))/10
            magMsg.magnetic_field.y = -(float(words[1])*mag_offset_y + mag_cali_y)/10
            magMsg.magnetic_field.z = (-(float(words[2])*mag_offset_z + mag_cali_z))/10

        magMsg.header.stamp= rospy.Time.now()
        magMsg.header.frame_id = frame_id
        magMsg.header.seq = seq_mag
        seq_mag = seq_mag + 1
        pub_mag.publish(magMsg)

    elif (line.find("#G-R=") != -1):
        line = line.replace("#G-R=","")   # Delete "#G-R="
        #f.write(line)                     # Write to the output log file
        line = line.replace("\r\n","")   # Delete "\r\n"
        words = line.split(",")    # Fields split
        if len(words) != 3:
            rospy.logwarn("Bad IMU data or bad sync")
            errcount = errcount+1
            continue
        else:
            errcount = 0
            imuMsg.angular_velocity.x = (float(words[0])-gyro_average_offset_x)
            imuMsg.angular_velocity.y = -(float(words[1])-gyro_average_offset_y)
            imuMsg.angular_velocity.z = -(float(words[2])-gyro_average_offset_z)
            ang_ok = 1

    elif (line.find("#A-R=") != -1):
        line = line.replace("#A-R=","")   # Delete "#A-R="
        #f.write(line)                     # Write to the output log file
        line = line.replace("\r\n","")   # Delete "\r\n"
        words = line.split(",")    # Fields split
        if len(words) != 3:
            rospy.logwarn("Bad IMU data or bad sync")
            errcount = errcount+1
            continue
        else:
            errcount = 0
            imuMsg.linear_acceleration.x = (-(float(words[0])*acc_offset_x + acc_cali_x)) * accel_factor
            imuMsg.linear_acceleration.y = (float(words[1])*acc_offset_y + acc_cali_y) * accel_factor
            imuMsg.linear_acceleration.z = (float(words[2])*acc_offset_z + acc_cali_z) * accel_factor
            acc_ok = 1
    
    if (acc_ok==1 and ang_ok==1):
        imuMsg.header.stamp= rospy.Time.now()
        imuMsg.header.frame_id = frame_id
        imuMsg.header.seq = seq_imu
        seq_imu = seq_imu + 1
        pub_imu.publish(imuMsg)
        acc_ok = 0;
        ang_ok = 0;
        
ser.close

#f.close

if (errcount > 10):
    sys.exit(10)
