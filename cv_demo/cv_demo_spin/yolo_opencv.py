# -*- coding: utf-8 -*-
# @Author: HeroHFM
# @Date:   2021-08-19 22:25:19
# @Last Modified by:   Victor Kamel
# @Last Modified time: 2021-08-25 17:17:52

# Source: Modified from Object detection - YOLO - OpenCV by Arun Ponnusamy (July 16, 2018)
# http://www.arunponnusamy.com

import cv2
import numpy as np

# FILES
CLASSES = 'yolov3.txt'          # Files from DARKNET YOLO - https://pjreddie.com/darknet/yolo/
WEIGHTS = 'yolov3-tiny.weights'
CONFIG  = 'yolov3-tiny.cfg'

class YOLOv3:
    def __init__(self):
        self.classes = None
        with open(CLASSES, 'r') as f: self.classes = [line.strip() for line in f.readlines()]

        self.COLORS = np.random.uniform(0, 255, size=(len(self.classes), 3))
        self.scale = 0.00392

        self.net = cv2.dnn.readNet(WEIGHTS, CONFIG)

    def gen_prediction(self, image, dest1, dest2):
        Height, Width = image.shape[:2]
        self.net.setInput(
            cv2.dnn.blobFromImage(image, self.scale, (416,416), (0,0,0), True, crop=False)
        )

        layer_names = self.net.getLayerNames()

        layer_list = [layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]

        outs = self.net.forward(
            [layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]
        )

        class_ids, confidences, boxes = [], [], []
        conf_threshold, nms_threshold = 0.5, 0.4

        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5:
                    center_x = int(detection[0] * Width)
                    center_y = int(detection[1] * Height)
                    w = int(detection[2] * Width)
                    h = int(detection[3] * Height)
                    x = center_x - w / 2
                    y = center_y - h / 2
                    class_ids.append(class_id)
                    confidences.append(float(confidence))
                    boxes.append([x, y, w, h])

        for i in cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold, nms_threshold):
            i = i[0]
            x, y, w, h = boxes[i][0:4]
            self.draw_prediction(dest1, 0, class_ids[i], confidences[i], round(x), round(y), round(x+w), round(y+h))
            self.draw_prediction(dest2, 1, class_ids[i], confidences[i], round(x), round(y), round(x+w), round(y+h))

        return dest1, dest2

    def draw_prediction(self, img, style, class_id, confidence, x, y, x_plus_w, y_plus_h):
        try: # FAILS WHEN BOUNDING BOX INTESECTS W/ EDGE OF FRAME, NEEDS TO BE FIXED
            label = str(self.classes[class_id])
            color = self.COLORS[class_id]
            cv2.rectangle(img, (x,y), (x_plus_w,y_plus_h), color, 3)

            if style:

                roi_c = img[y:y_plus_h, x:x_plus_w]
                mask = cv2.inRange(roi_c, (0, 0, 0), (55, 55, 55))
                roi_c[mask == 0] = color
                img[y:y_plus_h, x:x_plus_w] = roi_c # cv2.bitwise_not(roi_c)

                cv2.putText(img, label.upper(), (x,y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        except: pass
