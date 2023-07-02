#!/usr/bin/env python

import rospy
import serial
import string
import math
import sys

#from time import time
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
from dynamic_reconfigure.server import Server
from razor_imu_9dof.cfg import imuConfig
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from razor_imu_9dof.msg import orientation
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
orient = orientation()
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
port = rospy.get_param('~port', '/dev/ttyACM0')
topic = rospy.get_param('~topic', 'imu')
frame_id = rospy.get_param('~frame_id', 'base_imu_link')

# read calibration parameters

# accelerometer
accel_x_min = rospy.get_param('~accel_x_min', -269.90)
accel_x_max = rospy.get_param('~accel_x_max', 279.42)
accel_y_min = rospy.get_param('~accel_y_min', -258.79)
accel_y_max = rospy.get_param('~accel_y_max', 267.82)
accel_z_min = rospy.get_param('~accel_z_min', -308.72)
accel_z_max = rospy.get_param('~accel_z_max', 78.25)

# magnetometer
magn_x_min = rospy.get_param('~magn_x_min', -339.08)
magn_x_max = rospy.get_param('~magn_x_max', 816.20)
magn_y_min = rospy.get_param('~magn_y_min', -330.08)
magn_y_max = rospy.get_param('~magn_y_max', 780.20)
magn_z_min = rospy.get_param('~magn_z_min', -379.59)
magn_z_max = rospy.get_param('~magn_z_max', 652.66)
calibration_magn_use_extended = rospy.get_param('~calibration_magn_use_extended', False)
magn_ellipsoid_center = rospy.get_param('~magn_ellipsoid_center', [0, 0, 0])
magn_ellipsoid_transform = rospy.get_param('~magn_ellipsoid_transform', [[0, 0, 0], [0, 0, 0], [0, 0, 0]])
imu_yaw_calibration = rospy.get_param('~imu_yaw_calibration', 0.0)

# gyroscope
gyro_average_offset_x = rospy.get_param('~gyro_average_offset_x', -0.01)
gyro_average_offset_y = rospy.get_param('~gyro_average_offset_y', 0.0)
gyro_average_offset_z = rospy.get_param('~gyro_average_offset_z', -0.05)

#rospy.loginfo("%f %f %f %f %f %f", accel_x_min, accel_x_max, accel_y_min, accel_y_max, accel_z_min, accel_z_max)
#rospy.loginfo("%f %f %f %f %f %f", magn_x_min, magn_x_max, magn_y_min, magn_y_max, magn_z_min, magn_z_max)
#rospy.loginfo("%s %s %s", str(calibration_magn_use_extended), str(magn_ellipsoid_center), str(magn_ellipsoid_transform[0][0]))
#rospy.loginfo("%f %f %f", gyro_average_offset_x, gyro_average_offset_y, gyro_average_offset_z)

pub = rospy.Publisher(topic, Imu, queue_size=1)
orient_pub = rospy.Publisher("orientation", orientation, queue_size=1)
srv = Server(imuConfig, reconfig_callback)  # define dynamic_reconfigure callback
diag_pub = rospy.Publisher('diagnostics', DiagnosticArray, queue_size=1)
diag_pub_time = rospy.get_time();

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
    if ((line.find("#YPRAG=") == -1) or (line.find("\r\n") == -1)): 
        rospy.logwarn("Bad IMU data or bad sync")
        errcount = errcount+1
        continue
    line = line.replace("#YPRAG=","")   # Delete "#YPRAG="
    #f.write(line)                     # Write to the output log file
    line = line.replace("\r\n","")   # Delete "\r\n"
    words = line.split(",")    # Fields split
    if len(words) != 9:
        rospy.logwarn("Bad IMU data or bad sync")
        errcount = errcount+1
        continue
    else:
        errcount = 0
        #in AHRS firmware z axis points down, in ROS z axis points up (see REP 103)
        yaw_deg = -float(words[0])
        yaw_deg = yaw_deg + imu_yaw_calibration
        #if yaw_deg > 179.0:
            #yaw_deg = yaw_deg + 360.0
        if yaw_deg > -180.0 and yaw_deg<0 :
            yaw_deg = yaw_deg + 360.0
        #yaw_deg = 0.0
        yaw = yaw_deg*degrees2rad
        print("yaw",yaw)
        #in AHRS firmware y axis points right, in ROS y axis points left (see REP 103)
        pitch = -float(words[1])*degrees2rad
        roll = float(words[2])*degrees2rad

        # Publish message
        # AHRS firmware accelerations are negated
        # This means y and z are correct for ROS, but x needs reversing
        imuMsg.linear_acceleration.x = -float(words[3]) * accel_factor
        imuMsg.linear_acceleration.y = float(words[4]) * accel_factor
        imuMsg.linear_acceleration.z = float(words[5]) * accel_factor

        imuMsg.angular_velocity.x = float(words[6])
        #in AHRS firmware y axis points right, in ROS y axis points left (see REP 103)
        imuMsg.angular_velocity.y = -float(words[7])
        #in AHRS firmware z axis points down, in ROS z axis points up (see REP 103) 
        imuMsg.angular_velocity.z = -float(words[8])

    q = quaternion_from_euler(roll,pitch,yaw)
    orient.roll = roll
    orient.pitch = pitch
    orient.yaw = yaw
    imuMsg.orientation.x = q[0]
    imuMsg.orientation.y = q[1]
    imuMsg.orientation.z = q[2]
    imuMsg.orientation.w = q[3]
    print(q[3])
    imuMsg.header.stamp= rospy.Time.now()
    imuMsg.header.frame_id = frame_id
    imuMsg.header.seq = seq
    seq = seq + 1
    pub.publish(imuMsg)
    orient_pub.publish(orient)
    if (diag_pub_time < rospy.get_time()) :
        diag_pub_time += 1
        diag_arr = DiagnosticArray()
        diag_arr.header.stamp = rospy.get_rostime()
        diag_arr.header.frame_id = '1'
        diag_msg = DiagnosticStatus()
        diag_msg.name = 'Razor_Imu'
        diag_msg.level = DiagnosticStatus.OK
        diag_msg.message = 'Received AHRS measurement'
        diag_msg.values.append(KeyValue('roll (deg)',
                                str(roll*(180.0/math.pi))))
        diag_msg.values.append(KeyValue('pitch (deg)',
                                str(pitch*(180.0/math.pi))))
        diag_msg.values.append(KeyValue('yaw (deg)',
                                str(yaw*(180.0/math.pi))))
        diag_msg.values.append(KeyValue('sequence number', str(seq)))
        diag_arr.status.append(diag_msg)
        diag_pub.publish(diag_arr)
        
ser.close

#f.close

if (errcount > 10):
    sys.exit(10)
