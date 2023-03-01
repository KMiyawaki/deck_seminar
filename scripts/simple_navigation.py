#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion


def goto_point(ac, x, y, theta):
    # ナビゲーションする関数
    # 第1引数：生成済みAction Client
    # 第2,3引数：目標の座標（単位：メートル）
    # 第4引数：ゴール到着時の姿勢（単位：ラジアン）
    # ゴールの生成
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()  # 現在時刻

    goal.target_pose.pose.position.x = x  # ★ ゴールのx座標
    goal.target_pose.pose.position.y = y  # ★ ゴールのy座標
    q = quaternion_from_euler(0, 0, theta)  # ★ ゴール到着時に向かせたい方向
    goal.target_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

    rospy.loginfo('Sending goal')
    ac.send_goal(goal)  # ★ ゴールを送信する。
    finished = ac.wait_for_result(rospy.Duration(30))  # ★ 何等かの結果が出るまで待つ（30秒以内）
    state = ac.get_state()
    if finished:
        rospy.loginfo('Finished: (%d)', state)
    else:
        rospy.loginfo('Time out: (%d)', state)


def main():
    rospy.init_node('simple_navigation')
    # ★ Action Client
    ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # ★ アクションサーバーが起動するまで待つ。引数はタイムアウトの時間(秒）
    while not ac.wait_for_server(rospy.Duration(5)):
        rospy.loginfo('Waiting for the move_base action server to come up')

    rospy.loginfo('The server comes up')
    goto_point(ac, 7, 3, math.radians(90))  # ★ 座標（7,3）に90度の向きで止まるようにナビゲーションする


if __name__ == '__main__':
    main()
