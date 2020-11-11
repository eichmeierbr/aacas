#!/usr/bin/env python

from __future__ import division

from models import *
from utils.utils import *
from utils.datasets import *
from sort import iou, Tracklet
from detector import Detector

import cv2
import numpy as np
import os, sys

import torch
import rospy
from rospkg import RosPack
from cv_bridge import CvBridge, CvBridgeError

package = RosPack()
package_path = package.get_path('yolov3_sort')

from sensor_msgs.msg import Image
from yolov3_sort.msg import BoundingBox, BoundingBoxes

class Tracker():
    def __init__(self):
        weights_name = rospy.get_param('~weights_name')
        self.weights_path = os.path.join(package_path, 'weights', weights_name)
        if not os.path.isfile(self.weights_path):
            raise IOError("weights not found :(")

        config_name = rospy.get_param('~config_name', 'yolov3-custom.cfg')
        self.config_path = os.path.join(package_path, 'config', config_name)
        classes_name = rospy.get_param('~classes_name', 'custom.names')
        self.class_path = os.path.join(package_path, 'config', classes_name)

        self.class_list = []
        f = open(self.class_path, "r")
        for x in f:
            self.class_list.append(x.rstrip())

        self.conf_thres = rospy.get_param('~confidence', 0.9)
        self.nms_thres = rospy.get_param('~nms', 0.1)
        self.img_size = 416

        # Load network
        self.yolo = Detector(self.config_path,
                             self.weights_path,
                             self.class_path,
                             self.conf_thres,
                             self.nms_thres,
                             self.img_size)

        self.tracklets = []
        self.colors = []
        self.object_count = 0
        rospy.loginfo("Loaded Darknet for YOLO object detection.")

        # Define subscriber & callback function
        self.bridge = CvBridge()
        self.image_topic = rospy.get_param('~image_topic', '/camera/rgb/image_raw')
        self.sub = rospy.Subscriber(self.image_topic, Image, self.imageCb, queue_size=1, buff_size = 2**24)

        # Define publisher & topics
        self.tracked_objects_topic = rospy.get_param('~tracked_objects_topic')
        self.published_image_topic = rospy.get_param('~published_image_topic')

        self.pub = rospy.Publisher(self.tracked_objects_topic, BoundingBoxes, queue_size=10)
	self.raw_pub = rospy.Publisher("/raw_bboxes", BoundingBoxes, queue_size=10)
        self.img_pub = rospy.Publisher(self.published_image_topic, Image, queue_size=10)
        rospy.loginfo("Launched node for object tracking.")

        rospy.spin()

    def imageCb(self, data):
        try:
            #convert to opencv
            cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
        except CvBridgeError as e:
            print(e)

        # Store tracking results
        raw_bboxes = BoundingBoxes()
        raw_bboxes.header = data.header
        raw_bboxes.image_header = data.header

        yolo_boxes = self.yolo.detect(cv_image)

        for [x, y, w, h, c] in yolo_boxes:
            bbox_msg = BoundingBox()
            bbox_msg.xmin = x
            bbox_msg.ymin = y
            bbox_msg.xmax = x + w
            bbox_msg.ymax = y + h
            bbox_msg.label = c
            bbox_msg.idx = 0
            raw_bboxes.bounding_boxes.append(bbox_msg)

        bboxes = BoundingBoxes()
        bboxes.header = data.header
        bboxes.image_header = data.header

        for tr in self.tracklets:
            tr.setActive(False)

        for yolo_box in yolo_boxes:
            match = 0
            for tr in self.tracklets:
                if yolo_box[-1] != tr.label:
                    continue

                if iou(yolo_box[:4], tr.getState()):
                    tr.update(yolo_box[:4])
                    match = 1
                    tr.setActive(True)
                    tr.zeroTimeout()
                    break

            if match==0:
                tr = Tracklet(self.object_count, yolo_box[-1], yolo_box[:-1])
                self.tracklets.append(tr)
                self.object_count += 1
                self.colors.append((np.random.randint(0, 255, 3)))

        for tr in self.tracklets:
            if tr.active==False:
                tr.addTimeout()

            #if tr.active:
            if True:
                (x, y, w, h) = tr.getState()
                bbox_msg = BoundingBox()
                if tr.timeout > 2:
                    bbox_msg.label = -1

                else:
                    bbox_msg.label = tr.label

                bbox_msg.xmin = x
                bbox_msg.ymin = y
                bbox_msg.xmax = x + w
                bbox_msg.ymax = y + h
                #bbox_msg.label = tr.label
                bbox_msg.idx = tr.idx
                bboxes.bounding_boxes.append(bbox_msg)

                if tr.timeout > 2:
                    self.tracklets.remove(tr)

                else:
                    tr.predict()
                    #tr.addLength()
                    #rospy.loginfo(str(tr.length))

        self.pub.publish(bboxes)
	self.raw_pub.publish(raw_bboxes)
        self.visualize(bboxes, cv_image)

        return True

    def visualize(self, msg, data):
        img = data.copy()
        font = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 0.6
        thickness = 2

        if len(msg.bounding_boxes):
            for i in range(len(msg.bounding_boxes)):
                label = int(msg.bounding_boxes[i].label)
                x1 = msg.bounding_boxes[i].xmin
                y1 = msg.bounding_boxes[i].ymin
                x2 = msg.bounding_boxes[i].xmax
                y2 = msg.bounding_boxes[i].ymax
                w, h = x2-x1, y2-y1
                if label != -1:
                    color = self.colors[msg.bounding_boxes[i].idx]

                    cv2.rectangle(img,
                                  (int(x1), int(y1)),
                                  (int(x2), int(y2)),
                                  (color[0], color[1], color[2]), thickness)

                    text = self.class_list[label]
                    cv2.putText(img, text,
                                (int(x1), int(y1)-10),
                                font, fontScale, (0, 0, 0), thickness, cv2.LINE_AA)

        img_msg = self.bridge.cv2_to_imgmsg(img, "rgb8")
        self.img_pub.publish(img_msg)

if __name__=="__main__":
    rospy.init_node("tracker_node")
    TR = Tracker()
