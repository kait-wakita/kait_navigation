#!/usr/bin/env python

########################################################################################
# simple odometory publisher
#    subscribe: /cmd_vel
#    publish:   /odom      (use only /cmd_vel)
########################################################################################

import sys, rospy, math, tf
from geometry_msgs.msg import Twist, Quaternion, TransformStamped, Point
from nav_msgs.msg import Odometry

class PubOdom():
    def __init__(self):
        self.last_time = rospy.Time.now()
        
        self.pub_odom = rospy.Publisher('odom', Odometry, queue_size=10)
        self.bc_odom = tf.TransformBroadcaster()

        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.vx, self.vth = 0.0, 0.0

        self.cur_time = rospy.Time.now()
        self.last_time = self.cur_time

        
    def callback_cmd_vel(self,message):
        self.vx = message.linear.x
        self.vth = message.angular.z
        self.last_time = rospy.Time.now()


    def send_odom(self):
        self.cur_time = rospy.Time.now()

        dt=self.cur_time.to_sec() - self.last_time.to_sec()
        self.x += self.vx * math.cos(self.th) * dt
        self.y += self.vx * math.sin(self.th) * dt
        self.th += self.vth * dt

        q = tf.transformations.quaternion_from_euler(0, 0, self.th)
        self.bc_odom.sendTransform((self.x,self.y,0.0), q, self.cur_time, "base_link", "odom")

        odom = Odometry()
        odom.header.stamp = self.cur_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position = Point(self.x, self.y, 0)
        odom.pose.pose.orientation = Quaternion(*q)

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.vth

        self.pub_odom.publish(odom)

        self.last_time = self.cur_time
        

    
if __name__ == '__main__':
    rospy.init_node('pub_odom_dumb')
    m = PubOdom()
    
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        m.send_odom()
        rate.sleep()

