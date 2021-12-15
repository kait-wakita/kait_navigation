#!/usr/bin/env python

########################################################################################
# odometoy calc 
#    subscribe: /cmd_vel /gyro
#    publish:   /odom      (use /gyro yaw rate and theta)
########################################################################################

import sys, rospy, math, tf
from geometry_msgs.msg import Twist, Quaternion, TransformStamped, Point
from nav_msgs.msg import Odometry

class PubOdomGyro():
    def __init__(self):
        self.sub_cmd_vel = rospy.Subscriber('cmd_vel', Twist, self.callback_cmd_vel)
        self.sub_gyro = rospy.Subscriber('gyro', Twist, self.callback_gyro)
        self.pub_odom = rospy.Publisher('odom', Odometry, queue_size=10)
        self.bc_odom = tf.TransformBroadcaster()

        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.vx, self.vy, self.vth = 0.0, 0.0, 0.0
        self.gyro_z, self.gyro_th = 0.0, 0.0
        self.th_last = 0.0

        self.cur_time = rospy.Time.now()
        self.last_time = self.cur_time

        
    def callback_cmd_vel(self,message):
        self.vx = message.linear.x
        self.vy = message.linear.y
        self.vth = message.angular.z


    def callback_gyro(self,message):
        self.gyro_z = message.angular.z * 3.14 / 180
        self.gyro_th = message.linear.z * 3.14 / 180 # special mpu6050 node (mpu6050_gyro_talker.py) required

        
    def send_odom(self):
        self.cur_time = rospy.Time.now()

        dt=self.cur_time.to_sec() - self.last_time.to_sec()
        self.x += self.vx * math.cos(self.th) * dt
        self.y += self.vx * math.sin(self.th) * dt
        #self.th += self.th * self.vth * dt
        self.th = self.gyro_th

        q = tf.transformations.quaternion_from_euler(0, 0, self.th)
        self.bc_odom.sendTransform((self.x,self.y,0.0), q, self.cur_time, "base_link", "odom")

        odom = Odometry()
        odom.header.stamp = self.cur_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position = Point(self.x, self.y, 0)
        odom.pose.pose.orientation = Quaternion(*q)

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = (self.th-self.th_last)/dt

        self.pub_odom.publish(odom)

        self.last_time = self.cur_time
        self.th_last = self.th
        

    
if __name__ == '__main__':
    rospy.init_node('pub_odom_gyro')
    m = PubOdomGyro()

    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        m.send_odom()
        rate.sleep()

