#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import cv2

def on_mouse_click(event, x, y, flags, frame):
    if event == cv2.EVENT_LBUTTONDOWN:
        values = frame[y,x].tolist()
        print("x:%d, y:%d, HSV:" % (x, y) + str(values))


def main():
    if len(sys.argv) < 2:
        print('usage:' + sys.argv[0] + ' image_file [scale]')
        exit(1)
    file_name = sys.argv[1]
    scale = 0.5
    if len(sys.argv) >= 3:
        scale = float(sys.argv[2])
    img = cv2.imread(file_name)
    img = cv2.resize(img, dsize=None, fx=scale, fy=scale)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    cv2.imshow(file_name, img)
    cv2.setMouseCallback(file_name, on_mouse_click, hsv)
    cv2.waitKey(0)


if __name__ == '__main__':
    main()
