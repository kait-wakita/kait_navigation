#!/usr/bin/env python
# -*- coding: utf-8 -*-

########################################################################################
#2021卒研　歩行者ロボットプログラム ped1.py
#AMCLを使用し障害物回避せず目的地まで経路生成
#ベースプログラムMATLAB自律移動勉強会 navi_simple
########################################################################################

import rospy
import math
import numpy
import tf2_ros
import tf
import time
from geometry_msgs.msg import Twist


class pedestrian ():
    def __init__(self):
        self.robot_type = 'megarover'
        #tf取得
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        if self.robot_type == 'megarover_gazebo': #cmd_velのパブリッシャ設定
            self.twist_pub = rospy.Publisher('/vmegarover/diff_drive_controller/cmd_vel', Twist, queue_size=1)
        else:
            self.twist_pub = rospy.Publisher('/pedestrian1/cmd_vel', Twist, queue_size = 1)

    def goal(self):
        # 目的地の設定(xt=目標x座標, yt=目標y座標, zt=目標静止角度)
        xt = 6.0
        yt = 0.0
        zt = 170

        rate = rospy.Rate(10.0)

        while not rospy.is_shutdown():
            try:
                origin2base = self.tfBuffer.lookup_transform('map', 'pedestrian1/base_link', rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn( 'tf not found' )

            x = origin2base.transform.translation.x
            y = origin2base.transform.translation.y
            quaternion = origin2base.transform.rotation
            #クォータニオン（四元数）からオイラー角へ変換
            eul_d = numpy.rad2deg(tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))) 
        
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
        #停止後にオドメトリのYaw角度(旋回角度)が3.14ラジアン(180度)を超えるまで、正方向に旋回する
        while not rospy.is_shutdown():
            try:
                origin2base = self.tfBuffer.lookup_transform('map', 'pedestrian1/base_link', rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn( 'tf not found' )

            x = origin2base.transform.translation.x
            y = origin2base.transform.translation.y
            quaternion = origin2base.transform.rotation
            #クォータニオン（四元数）からオイラー角へ変換
            eul_d = numpy.rad2deg(tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w)))
            if zt < eul_d[2] :
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

    def start(self):
        # 初期位置の設定
        xt = 0.0
        yt = 0.0
        zt_max = 10
        zt_min = -10

        rate = rospy.Rate(10.0)

        while not rospy.is_shutdown():
            try:
                origin2base = self.tfBuffer.lookup_transform('map', 'pedestrian1/base_link', rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn( 'tf not found' )

            x = origin2base.transform.translation.x
            y = origin2base.transform.translation.y
            quaternion = origin2base.transform.rotation
            #クォータニオン（四元数）からオイラー角へ変換
            eul_d = numpy.rad2deg(tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))) 
        
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
        #停止後にオドメトリのYaw角度(旋回角度)が0ラジアン(0度)を超えるまで、正方向に旋回する
        while not rospy.is_shutdown():
            try:
                origin2base = self.tfBuffer.lookup_transform('map', 'pedestrian1/base_link', rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn( 'tf not found' )

            x = origin2base.transform.translation.x
            y = origin2base.transform.translation.y
            quaternion = origin2base.transform.rotation
            #クォータニオン（四元数）からオイラー角へ変換
            eul_d = numpy.rad2deg(tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w)))
            if zt_min < eul_d[2] < zt_max:
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
            P.goal()
        if 's' in direction:
            P.start()
    rate.sleep()

