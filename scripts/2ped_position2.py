#!/usr/bin/env python
# -*- coding: utf-8 -*-

########################################################################################
#2021卒研　歩行者ロボット2台走行プログラム 2ped_position2.py
#AMCLを使用し障害物回避せず目的地まで経路生成
#ベースプログラムMATLAB自律移動勉強会 navi_simple
#2021/10/20 2ped_1a　2台それぞれ走行を作成
#2021/10/22 2ped_1b　pedestrian2/base_link追加、tf not found修正rospy.Dulation追加
#2021/10/22 2ped_2a  2台同時タイミング走行　threading使用 失敗
#2021/10/22 2ped_3a　プログラム練り直し　ped1しか走行せず
#2021/10/26 2ped_3b  2台走行も2手に分かれて走行する
#2021/10/26 2ped_3c  原因調査のため速度0.1m/sに変更
#2021/10/26 2ped_3d  ped1とped2の間にtime.sleep追加で走行不良を解決
#2021/10/26 2ped_3e  目的地到着判定条件を0.15から0.5に緩和
#2021/10/26 2ped_position1、2ped_position2に名称変更
########################################################################################

import rospy, math, numpy, tf2_ros, tf, time
from geometry_msgs.msg import Twist

class pedestrian ():
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.twist_pub_ped1 = rospy.Publisher('/pedestrian1/cmd_vel', Twist, queue_size = 1)
        self.twist_pub_ped2 = rospy.Publisher('/pedestrian2/cmd_vel', Twist, queue_size = 1)

    def nav(self,xt,yt,zt_min,zt_max,robot_name):
        def get_tf():
            if robot_name == "pedestrian1":
                try:
                    origin2base = self.tfBuffer.lookup_transform('map', 'pedestrian1/base_link', rospy.Time(0), rospy.Duration(10.0))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logwarn( 'tf not found' )

            elif robot_name == "pedestrian2":
                try:
                    origin2base = self.tfBuffer.lookup_transform('map', 'pedestrian2/base_link', rospy.Time(0), rospy.Duration(10.0))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logwarn( 'tf not found' )

            global x,y,eul_d
            x = origin2base.transform.translation.x
            y = origin2base.transform.translation.y
            quaternion = origin2base.transform.rotation
            #クォータニオン（四元数）からオイラー角へ変換
            eul_d = numpy.rad2deg(tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w)))

        def twist_pub():
            if robot_name == "pedestrian1":
                self.twist_pub_ped1.publish(twist)
            elif robot_name == "pedestrian2":
                self.twist_pub_ped2.publish(twist)

        #現在地取得
        get_tf()
        
        #回転方向演算
        r = math.atan2(yt-y, xt-x)
        rt = numpy.rad2deg(r) - eul_d[2]

        #ラジアン範囲正規化(-pi~pi)
        dir_goal = (rt + 180) % 360 - 180

        #目的地到着していたら停止後に回転
        if(math.sqrt((xt-x)**2+(yt-y)**2)<0.50):
            print(robot_name,xt,yt,'reached goal')
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            twist_pub()
            #回転
            get_tf()
            if zt_min < eul_d[2] <zt_max :
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                twist_pub()
            else:
                #デバッグ用にオドメトリのYaw角度(旋回角度)を表示
                print(robot_name,eul_d[2])
                twist.linear.x = 0.0
                twist.angular.z = 0.5
                twist_pub()
        else:
            #デバッグ用に現在座標位置を表示
            print(robot_name,x,y,eul_d[2])
            twist = Twist()
            twist.linear.x = 0.5
            #numpy.deg2radで度数からラジアンに変換
            twist.angular.z = 0.5 * (numpy.deg2rad(dir_goal))
            twist_pub()

if __name__ == '__main__':
    rospy.init_node('pedestrian')
    P = pedestrian()
    while not rospy.is_shutdown():
        P.nav(6, 0, 170, 180, "pedestrian1"),
        time.sleep(0.1)
        P.nav(6, -1.5, 170, 180, "pedestrian2")
    time.sleep(0.1)
