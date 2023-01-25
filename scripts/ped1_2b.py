#!/usr/bin/env python
# -*- coding: utf-8 -*-

########################################################################################
#2021卒研　歩行者ロボットプログラム ped1.py
#AMCLを使用し障害物回避せず目的地まで経路生成
#ベースプログラムMATLAB自律移動勉強会 navi_simple
#2021/10/20 ped1_2a 関数定義
########################################################################################

import rospy, math, numpy, tf2_ros, tf, time
from geometry_msgs.msg import Twist

class pedestrian ():
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.twist_pub = rospy.Publisher('/pedestrian1/cmd_vel', Twist, queue_size = 1)

    def nav(self,xt,yt,zt_min,zt_max):
        def get_tf():
            try:
                origin2base = self.tfBuffer.lookup_transform('map', 'pedestrian1/base_link', rospy.Time(0), rospy.Duration(10.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn( 'tf not found' )

            global x,y,eul_d
            x = origin2base.transform.translation.x
            y = origin2base.transform.translation.y
            quaternion = origin2base.transform.rotation
                #クォータニオン（四元数）からオイラー角へ変換
            eul_d = numpy.rad2deg(tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w)))

        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            get_tf()
            #回転方向演算
            r = math.atan2(yt-y, xt-x)

            rt = numpy.rad2deg(r) - eul_d[2]

            #ラジアン範囲正規化(-pi~pi)
            dir_goal = (rt + 180) % 360 - 180

            #目的地到着したらループを抜ける
            if(math.sqrt((xt-x)**2+(yt-y)**2)<0.15):
                print(xt,yt,'reached goal')
                break
        
            #デバッグ用に現在座標位置を表示
            print(x,y,eul_d[2])
            twist = Twist()
            twist.linear.x = 0.5
            #numpy.deg2radで度数からラジアンに変換
            twist.angular.z = 0.5 * (numpy.deg2rad(dir_goal))
            self.twist_pub.publish(twist)
            time.sleep(0.1)

	    #停止
    	twist = Twist()
    	twist.linear.x = 0.0
    	twist.angular.z = 0.0
        self.twist_pub.publish(twist)
        #停止後に旋回
        while not rospy.is_shutdown():
            get_tf()
            if zt_min < eul_d[2] <zt_max :
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.twist_pub.publish(twist)
                break
            else:
                #デバッグ用にオドメトリのYaw角度(旋回角度)を表示
                print(eul_d[2])
                twist.linear.x = 0.0
                twist.angular.z = 0.5
                self.twist_pub.publish(twist)
            time.sleep(0.02)

if __name__ == '__main__':
    rospy.init_node('pedestrian')
    P = pedestrian()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        direction = raw_input('g: goal, s: start')
        if 'g' in direction:
            P.nav(6, 0, 170, 180)
        if 's' in direction:
            P.nav(0, 0, -10, 10)
    rate.sleep()
