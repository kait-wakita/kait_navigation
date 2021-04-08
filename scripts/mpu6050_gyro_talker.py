#! /usr/bin/env python
# coding:utf-8

# talker for mpu6050 gyro
#     read the value of mpu6050, publish the valude to '/gyro'
#        (message type 'Twist' is used for easy communication with MATLAB)
#     special: publish theta (integral yaw) to /gyro/linear/z
#

import smbus
import math
import time
import rospy
from sensor_msgs.msg import *
#from geometry_msgs.msg import *
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse

##############################################################
#アドレスの設定
##############################################################
DEV_ADDR = 0x68

ACCEL_XOUT = 0x3b
ACCEL_YOUT = 0x3d
ACCEL_ZOUT = 0x3f
TEMP_OUT = 0x41
GYRO_XOUT = 0x43
GYRO_YOUT = 0x45
GYRO_ZOUT = 0x47

PWR_MGMT_1 = 0x6b # set clock
PWR_MGMT_2 = 0x6c



bus = smbus.SMBus(8)  # bus number (common:1, jetson NX:8)
bus.write_byte_data(DEV_ADDR, PWR_MGMT_1, 0)
bus.write_byte_data(DEV_ADDR, 0x1B, 0x10)       # expand angle scale (0x00:250, 0x08:500, 0x10:1000, 0x18:2000deg/s)

def read_word(adr):
    high = bus.read_byte_data(DEV_ADDR, adr)
    low = bus.read_byte_data(DEV_ADDR, adr+1)
    val = (high << 8) + low
    return val

def read_word_sensor(adr):
    val = read_word(adr)
    if (val >= 0x8000):#minus
        return -((65535 - val) + 1)
    else:#plus
        return val

def getAccel():
    # unit is 'g'
    x = read_word_sensor(ACCEL_XOUT)/16384.0
    y = read_word_sensor(ACCEL_YOUT)/16384.0
    z = read_word_sensor(ACCEL_ZOUT)/16384.0
    return [x, y, z]

def getGyro():
    # unit is 'deg/sec'
    gx = read_word_sensor(GYRO_XOUT)/32.8
    gy = read_word_sensor(GYRO_YOUT)/32.8
    gz = read_word_sensor(GYRO_ZOUT)/32.8
    return [gx, gy, gz]

def calcEuler(x,y,z):
    theta = math.atan( x / math.sqrt(y*y + z*z) )
    psi = math.atan( y / math.sqrt(x*x + z*z) )
    phi = math.atan( math.sqrt( x*x + y*y ) / z)
    return [theta, psi, phi]


if __name__ == "__main__":
    rospy.init_node("gyro_talker")
    gyro_pub = rospy.Publisher('gyro',Twist,queue_size=1)
    twist = Twist()
    last_time = rospy.Time.now()
    last_published_t = last_time.to_sec()
    theta = 0

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        #ax, ay, az = getAccel()
        gx, gy, gz = getGyro()
        #roll, pitch, yaw = calcEuler(ax,ay,az)

        cur_time = rospy.Time.now()
        dt = cur_time.to_sec() - last_time.to_sec()

        gz = gz  + 0.00           # gyro offset adjust (require calibration)
        theta = theta + gz*dt

        twist.linear.z = theta    # special for this module, integral at high frequecny for precise theta calc.
        twist.angular.z = gz

        if cur_time.to_sec() > last_published_t+0.05:
            gyro_pub.publish(twist)
            last_published_t= cur_time.to_sec()

        last_time = cur_time

        # print "ACC[%5.2f,"%ax, "%5.2f,"%ay, "%5.2f]"%az, "   GYRO[%5.0f,"%gx, "%5.0f,"%gy, "%5.0f]"%gz

        rate.sleep()
   
