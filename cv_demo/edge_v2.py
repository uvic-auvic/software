# -*- coding: utf-8 -*-
# @Author: HeroHFM
# @Date:   2021-08-19 22:25:19
# @Last Modified by:   Victor Kamel
# @Last Modified time: 2021-08-20 11:14:36

import cv2
from yolo_opencv import YOLOv3

# GUI
GUI_TITLE = 'AUVIC Computer Vision Demo'
GUI_QUIT  = 'q'

# COMPOSITION PARAMETERS
X_OFF, Y_OFF = 465, 25

# EDGE DETECT PARAMETERS
low_threshold = 55 # 0 - 100
ratio = 3
kernel_size = 3


stream = cv2.VideoCapture()
stream.open(0, cv2.CAP_DSHOW)

cv2.namedWindow(GUI_TITLE, cv2.WINDOW_FREERATIO)
cv2.setWindowProperty(GUI_TITLE, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

def compose_image_offset(l_img, s_img, x_offset, y_offset):
	l_img[y_offset:y_offset+s_img.shape[0], x_offset:x_offset+s_img.shape[1]] = s_img

model = YOLOv3()

while True:
    ret, frame = stream.read()

    # CANNY EDGE DETECT
    wg = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # GRAYSCALE
    wb = cv2.blur(wg, (3,3))                     # BLURRED
    we = cv2.Canny(wb, low_threshold, low_threshold * ratio, kernel_size) # EDGE DETECT

    mask = we != 0
    res = frame * (mask[:,:,None].astype(frame.dtype))

    # SHOW IMAGE
    sub, res = model.gen_prediction(frame, frame.copy(), res)
    pip = cv2.resize(sub, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_CUBIC)
    compose_image_offset(res, pip, X_OFF, Y_OFF)

    cv2.imshow(GUI_TITLE, res)
    
    if cv2.waitKey(1) & 0xFF == ord(GUI_QUIT):  break

stream.release()
cv2.destroyAllWindows()
