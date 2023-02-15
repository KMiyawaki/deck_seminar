#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import sys
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class RobotController(object):
    def __init__(self):
        # 速度制御用
        # http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html
        # ★ トピック名'/cmd_vel'、型Twistのパブリッシャを生成する。
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # 画像処理用
        # http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html
        # ★ トピック名'/image_processing/result_image'、型Imageのパブリッシャを生成する。
        self.pub_img = rospy.Publisher('/image_processing/result_image', Image, queue_size=10)
        # ★ トピック名 '/video_source/raw'、型 Image のサブスクライバを生成する。
        # rospy.Subscriber('/video_source/raw', Image, self.image_cb) # ☆ 修正方法はセミナ中で説明します。
        self.cv_bridge = CvBridge()
        self.kernel = np.ones((5, 5), np.uint8)

    def test_run(self):
        # テスト走行
        func = sys._getframe().f_code.co_name
        rospy.loginfo('Executing ' + func)  # 実行中の関数名を表示する。
        rate = rospy.Rate(10)  # ループを10Hzで回す。
        # 前進
        start_time = rospy.get_time()  # 現在時刻を取得する。
        while rospy.get_time() - start_time < 2.0:  # 2秒間継続する。
            vel = Twist()  # ★ Twist型の変数velを定義する。ロボットへの速度指令が入る。
            vel.linear.x = 0.25  # ★ 0.25m／秒で前進する指令を作成する。
            self.pub_vel.publish(vel)  # ★ 速度指令をパブリッシュする。
            rate.sleep()  # ループを10Hzで回す。
        # 左旋回
        start_time = rospy.get_time()  # 現在時刻を取得する。
        while rospy.get_time() - start_time < 3.0:  # 3秒間継続する。
            vel = Twist()  # ★ Twist型の変数velを定義する。ロボットへの速度指令が入る。
            vel.angular.z = math.radians(30)  # ★ 30度／秒で左旋回する指令を作成する。
            self.pub_vel.publish(vel)  # ★ 速度指令をパブリッシュする。
            rate.sleep()  # ループを10Hzで回す。

    def extract_color(self, cv_image_in, scale):
        # 特定の色に当てはまる画素を抽出する。
        cv_image_in = cv2.resize(
            cv_image_in, dsize=None, fx=scale, fy=scale)  # 画像を縮小する。
        hsv = cv2.cvtColor(cv_image_in, cv2.COLOR_BGR2HSV)  # HSV形式に変換する。
        # ★ ここのパラメータを調整する。
        return cv2.inRange(hsv, (0, 40, 40), (10, 255, 255))

    def control(self, iw, ih, tx, ty, tw, th):
        # 領域追従のために速度制御する。
        # iw, ih 画像の幅、高さ
        # tx, ty 追従対象の中心座標
        # tw, th 追従対象の幅、高さ
        vel = Twist()
        self.pub_vel.publish(vel)

    def image_cb(self, msg):
        # 画像受信時に呼ばれる関数。
        scale = 0.25  # 画像を 1/4 に縮小して処理する。
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        cv_result = self.extract_color(cv_image, scale) # ★ 特定の色に当てはまる画素を抽出する。関数内のパラメータを調整する。
        cv_result, contours = self.detect_areas(cv_result, self.kernel)  # 抽出した画素の塊の輪郭を検出する。
        tx, ty, tw, th = self.tracking_target(cv_result, contours, 100)  # 輪郭を囲む矩形のうち、最も大きい矩形の中心とサイズを返す。
                                                                         # 十分な大きさの矩形が無かった場合は-1, -1, -1, -1を返す。
        if tx > 0:
            self.control(cv_result.shape[1],
                         cv_result.shape[0], tx, ty, tw, th)  # ★ この関数の内部を実装する
        else:
            self.stop()  # 追跡対象が無ければ車体を止める。
        self.pub_img.publish(
            self.cv_bridge.cv2_to_imgmsg(cv_result, "mono8"))

    def spin(self):
        # 一定時間ループする。
        rate = rospy.Rate(10)  # ループを10Hzで回す。
        start_time = rospy.get_time()  # 現在時刻を取得する。
        while rospy.get_time() - start_time < 120.0:  # 120秒間継続する。
            rate.sleep()

    def stop(self):
        # 速度ゼロをパブリッシュする
        self.pub_vel.publish(Twist())

    def detect_areas(self, cv_image_in, kernel):
        # 抽出した領域ごとに輪郭を検出する
        height, width = cv_image_in.shape
        cv_morph = np.zeros((height, width), dtype='uint8')
        cv_result = np.zeros((height, width), dtype='uint8')  # 処理結果の画像
        # http://labs.eecs.tottori-u.ac.jp/sd/Member/oyamada/OpenCV/html/py_tutorials/py_imgproc/py_morphological_ops/py_morphological_ops.html
        cv2.morphologyEx(cv_image_in, cv2.MORPH_OPEN,
                         kernel, cv_morph, iterations=2)
        cv_result[cv_morph > 127] = (128)  # 抽出された部分を塗りつぶす
        contours = None
        if cv2.__version__[0] == '3':
            _, contours, _ = cv2.findContours(
                cv_morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 輪郭を抽出する
        if cv2.__version__[0] == '4':
            contours, _ = cv2.findContours(
                cv_morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 輪郭を抽出する
        cv2.drawContours(cv_result, contours, -1, (255), 1)  # 輪郭を描画する
        return cv_result, contours  # 結果をタプルで返す

    def tracking_target(self, cv_image_in, contours, min_size):
        # 輪郭を囲む矩形のうち最大の矩形の中心座標と幅、高さを返す
        # 矩形面積が min_size 未満の場合は全ての要素が -1 のタプルを返す
        max, max_size = None, -1
        cx = cy = w = h = -1
        for c in contours:
            if len(c) == 0:
                continue
            x, y, w, h = cv2.boundingRect(c)  # 輪郭を囲む矩形を得る
            size = w * h
            if size < min_size:  # あまりにも面積が小さいものは除外する
                continue
            cv2.rectangle(cv_image_in, (x, y), (x + w, y + h),
                          (255), 1)  # 輪郭を囲む矩形を描画する
            if size > max_size:
                max, max_size = c, size
                cx, cy = x + w / 2, y + h / 2
        if max is not None:  # 最も大きな矩形の中心に丸印をつける
            cv2.circle(cv_image_in, (cx, cy), 5, (255), -1)
        return cx, cy, w, h  # 結果をタプルで返す


def main():
    user_name = 'Taro Robot'  # ご自身のお名前に変更してください。
    rospy.init_node('my_robot_controller')
    rospy.loginfo(user_name)  # 受講者の情報を表示する。
    rospy.sleep(1)  # 起動直後は rospy.get_time が異常な値を返すことがあるので。
    controller = RobotController()
    try:
        # テスト走行時
        controller.test_run() # ☆ 修正方法はセミナ中で説明します。
        # 画像処理＋ロボット制御時
        # controller.spin() # ☆ 修正方法はセミナ中で説明します。
        # controller.stop() # ☆ 修正方法はセミナ中で説明します。
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo('Finish ' + user_name)


if __name__ == '__main__':
    main()
