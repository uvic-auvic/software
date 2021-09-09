# -*- coding: utf-8 -*-
# @Author: HeroHFM
# @Date:   2021-08-19 22:25:19
# @Last Modified by:   Victor Kamel
# @Last Modified time: 2021-08-25 17:15:21

import cv2

# GUI
GUI_TITLE = 'AUVIC Computer Vision Demo'
GUI_QUIT  = 'q'

# COMPOSITION PARAMETERS
OFF_X, OFF_Y = 465, 25

# EDGE DETECT PARAMETERS
low_threshold = 55 # 0 - 100
ratio = 3
kernel_size = 3


stream = cv2.VideoCapture()
stream.open(0, cv2.CAP_DSHOW)

cv2.namedWindow(GUI_TITLE, cv2.WINDOW_FREERATIO)
# cv2.setWindowProperty(GUI_TITLE, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

def compose_image_offset(l_img, s_img, x_offset, y_offset):
	l_img[y_offset:y_offset+s_img.shape[0], x_offset:x_offset+s_img.shape[1]] = s_img

while True:
    ret, frame = stream.read()

    # CANNY EDGE DETECT
    wg = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # GRAYSCALE
    wb = cv2.blur(wg, (3,3))                     # BLURRED
    we = cv2.Canny(wb, low_threshold, low_threshold * ratio, kernel_size) # EDGE DETECT

    mask = we != 0
    res = frame * (mask[:,:,None].astype(frame.dtype))

    # SHOW IMAGE
    pip = cv2.resize(frame, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_CUBIC)
    compose_image_offset(res, pip, OFF_X, OFF_Y)
    cv2.imshow(GUI_TITLE, res)
    
    if cv2.waitKey(1) & 0xFF == ord(GUI_QUIT):  break

stream.release()
cv2.destroyAllWindows()

# import cv2

# stream = cv2.VideoCapture(0)

# if not stream.isOpened(): raise IOError("Failure(3) - Failed to open webcam.")

# while True:
#     ret, frame = stream.read()
#     frame = cv2.resize(frame, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
#     cv2.imshow('Input', frame)

#     c = cv2.waitKey(1)
#     if c == 27: break

# from __future__ import print_function
# import cv2 as cv
# import argparse
# max_lowThreshold = 100
# window_name = 'Edge Map'
# title_trackbar = 'Min Threshold:'
# ratio = 3
# kernel_size = 3
# def CannyThreshold(val):
#     low_threshold = val
#     img_blur = cv.blur(src_gray, (3,3))
#     detected_edges = cv.Canny(img_blur, low_threshold, low_threshold*ratio, kernel_size)
#     mask = detected_edges != 0
#     dst = src * (mask[:,:,None].astype(src.dtype))
#     cv.imshow(window_name, dst)
# parser = argparse.ArgumentParser(description='Code for Canny Edge Detector tutorial.')
# parser.add_argument('--input', help='Path to input image.', default='sample.jpg')
# args = parser.parse_args()
# src = cv.imread(cv.samples.findFile(args.input))
# if src is None:
#     print('Could not open or find the image: ', args.input)
#     exit(0)
# src_gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
# cv.namedWindow(window_name)
# cv.createTrackbar(title_trackbar, window_name , 0, max_lowThreshold, CannyThreshold)
# CannyThreshold(0)
# cv.waitKey()