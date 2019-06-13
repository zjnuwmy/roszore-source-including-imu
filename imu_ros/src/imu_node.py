#!/usr/bin/env python

from lsm6ds33 import LSM6DS33
import numpy
import rospy
import string
import math
import sys

#from time import time
from sensor_msgs.msg import Imu

degrees2rad = math.pi/180.0
imu_yaw_calibration = 0.0
_NEXT_AXIS = [1, 2, 0, 1]

_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
    'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
    'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
    'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
    'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
    'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
    'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
    'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

def quaternion_from_euler(ai, aj, ak, axes='sxyz'):
    """Return quaternion from Euler angles and axis sequence.

    ai, aj, ak : Euler's roll, pitch and yaw angles
    axes : One of 24 axis sequences as string or encoded tuple

    >>> q = quaternion_from_euler(1, 2, 3, 'ryxz')
    >>> numpy.allclose(q, [0.435953, 0.310622, -0.718287, 0.444435])
    True

    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        _TUPLE2AXES[axes]  # noqa: validation
        firstaxis, parity, repetition, frame = axes

    i = firstaxis + 1
    j = _NEXT_AXIS[i+parity-1] + 1
    k = _NEXT_AXIS[i-parity] + 1

    if frame:
        ai, ak = ak, ai
    if parity:
        aj = -aj

    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = numpy.empty((4, ))
    if repetition:
        q[0] = cj*(cc - ss)
        q[i] = cj*(cs + sc)
        q[j] = sj*(cc + ss)
        q[k] = sj*(cs - sc)
    else:
        q[0] = cj*cc + sj*ss
        q[i] = cj*sc - sj*cs
        q[j] = cj*ss + sj*cc
        q[k] = cj*cs - sj*sc
    if parity:
        q[j] *= -1.0

    return q



# Callback for dynamic reconfigure requests
rospy.init_node("razor_node")
#We only care about the most recent measurement, i.e. queue_size=1
pub = rospy.Publisher('thumper_imu', Imu, queue_size=1)
imuMsg = Imu()

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

#rospy.loginfo("%f %f %f", gyro_average_offset_x, gyro_average_offset_y, gyro_average_offset_z)
degrees2rad = math.pi/180.0
imu_yaw_calibration = 0.0
accel_factor = 9.806 / 256.0    # sensor reports accel as 256.0 = 1G (9.8m/s^2). Convert to m/s^2.


def getrpy(words):
    yaw_deg = -float(words[0])
    yaw_deg = yaw_deg + imu_yaw_calibration
    if yaw_deg > 180.0:
        yaw_deg = yaw_deg - 360.0
    if yaw_deg < -180.0:
        yaw_deg = yaw_deg + 360.0
 
    yaw = yaw_deg*degrees2rad
    #in AHRS firmware y axis points right, in ROS y axis points left (see REP 103)
    pitch = -float(words[1])*degrees2rad
    roll = float(words[2])*degrees2rad
    ret=[roll, pitch, yaw]
        
    return ret

roll=0
pitch=0
yaw=0
seq=0
accel_factor = 9.806 / 256.0    # sensor reports accel as 256.0 = 1G (9.8m/s^2). Convert to m/s^2.
rospy.loginfo("Giving the razor IMU board 5 seconds to boot...")
rospy.sleep(5) # Sleep for 5 seconds to wait for the board to boot

imu = LSM6DS33()
imu.enable()
while not rospy.is_shutdown():
    linvel = imu.getAccelerometer3Angles()
    ryp=getrpy(linvel)
    
    avel=imu.get_gyro_angular_velocity()
    q = quaternion_from_euler(ryp[0], ryp[1],ryp[2])
        # Publish message
        # AHRS firmware accelerations are negated
        # This means y and z are correct for ROS, but x needs reversing
    imuMsg.linear_acceleration.x = -linvel[0] 
    imuMsg.linear_acceleration.y = linvel[1] 
    imuMsg.linear_acceleration.z = linvel[2] 

    imuMsg.angular_velocity.x = avel[0]
    #in AHRS firmware y axis points right, in ROS y axis points left (see REP 103)
    imuMsg.angular_velocity.y = -avel[1]
    #in AHRS firmware z axis points down, in ROS z axis points up (see REP 103) 
    imuMsg.angular_velocity.z = -avel[2]

    q = quaternion_from_euler(ryp[0], ryp[1],ryp[2])
    imuMsg.orientation.x = q[0]
    imuMsg.orientation.y = q[1]
    imuMsg.orientation.z = q[2]
    imuMsg.orientation.w = q[3]
    imuMsg.header.stamp= rospy.Time.now()
    imuMsg.header.frame_id = 'base_imu_link'
    imuMsg.header.seq = seq
    seq = seq + 1
    pub.publish(imuMsg)

