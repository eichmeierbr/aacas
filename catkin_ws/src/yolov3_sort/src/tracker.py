#!/usr/bin/env python

from __future__ import division

from sort import iou, Tracklet

#from models import models 
import cv2
import numpy as np
import os, sys

import rospy
from rospkg import RosPack
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped, PointStamped, Point, Vector3, Quaternion

package = RosPack()
package_path = package.get_path('yolov3_sort')

from sensor_msgs.msg import Image
from yolov3_sort.msg import BoundingBox, BoundingBoxes
from lidar_process.msg import tracked_obj_arr, tracked_obj

class Tracker():
    def __init__(self):
        self.offline = True
        self.publish_raw_bboxes = False

        self.bboxes = []
        self.tracklets = []
        self.object_count = 25 ######25
        self.colors = []

	    # Model parameters
        classes_name = rospy.get_param('~classes_name', 'custom.names')
        self.class_path = os.path.join(package_path, 'config', classes_name)
        self.class_list = []
        f = open(self.class_path, "r")
        for x in f:
            self.class_list.append(x.rstrip())

        if not self.offline:
            from detector import Detector

    	    weights_name = rospy.get_param('~weights_name')
            self.weights_path = os.path.join(package_path, 'weights', weights_name)
            if not os.path.isfile(self.weights_path):
                raise IOError("weights not found :(")

            config_name = rospy.get_param('~config_name', 'yolov3-custom.cfg')
            self.config_path = os.path.join(package_path, 'config', config_name)
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

        self.drone_pos = np.zeros(3)
        self.quat = np.zeros(4)
        self.prev_trans = np.identity(4)
        self.curr_trans = np.identity(4)

        # Store extrinsic matrix & intrinsic matrix
        self.dT = np.identity(4)
        self.K = np.array([[574.0198,      0.0, 318.1983, 0.0],
                           [     0.0, 575.2453, 246.5657, 0.0],
                           [     0.0,      0.0,      1.0, 0.0]])
        # Store 3d points (center, homogeneous)
        self.XYZ = {}
        # Store control inputs
        self.u = {}

        # Define subscriber & callback function
        self.bridge = CvBridge()
        self.image_topic = rospy.get_param('~image_topic', '/camera/rgb/image_raw')
        self.img_sub = rospy.Subscriber(self.image_topic, Image, self.imageCb, queue_size=1, buff_size = 2**24)
        self.pos_sub = rospy.Subscriber('/dji_sdk/local_position', PointStamped, self.position_callback, queue_size=1)
        self.att_sub = rospy.Subscriber('/dji_sdk/attitude', QuaternionStamped, self.attitude_callback, queue_size=1)
        self.lidar_sub = rospy.Subscriber('/tracked_obj_pos_arr_local', tracked_obj_arr, self.lidar_callback, queue_size=1)

        # Define publisher & topics
        self.tracked_objects_topic = rospy.get_param('~tracked_objects_topic')
        self.published_image_topic = rospy.get_param('~published_image_topic')

        self.pub = rospy.Publisher(self.tracked_objects_topic, BoundingBoxes, queue_size=10)
        self.img_pub = rospy.Publisher(self.published_image_topic, Image, queue_size=10)

        if self.offline:
            self.bbox_topic = rospy.get_param('~bbox_topic', '/raw_bboxes')
            self.bbox_sub = rospy.Subscriber(self.bbox_topic, BoundingBoxes, self.bbox_callback, queue_size=1)

        if self.publish_raw_bboxes:
            self.raw_pub = rospy.Publisher("/raw_bboxes", BoundingBoxes, queue_size=10)

        rospy.loginfo("Launched node for object tracking.")
        rospy.spin()

    
    def bbox_callback(self, msg): # get detections from bag
        if not self.offline:
            pass
        else:
            self.bboxes = []
            for b in msg.bounding_boxes:
                self.bboxes.append(np.array([b.xmin, b.ymin, b.xmax-b.xmin, b.ymax-b.ymin, b.label]))

    def attitude_callback(self, msg):
        q = msg.quaternion
        self.quat = np.array([q.w, q.x, q.y, q.z])

    def position_callback(self, msg):
        pt = msg.point
        self.drone_pos = np.array([pt.x, pt.y, pt.z])

    def lidar_callback(self, msg):
        # object_id, point
        self.XYZ = {} # clear dictionary
        for obj in msg.tracked_obj_arr:
            idx = obj.object_id
            pt = obj.point
            self.XYZ[idx] = np.array([[pt.z], [-pt.y], [pt.x], [1.]])

    def yaw_pitch_roll(self):
        q0, q1, q2, q3 = self.quat[0],self.quat[1], self.quat[2], self.quat[3]
        q2sqr = q2 * q2
        t0 = -2.0 * (q2sqr + q3 * q3) + 1.0
        t1 = +2.0 * (q1 * q2 + q0 * q3)
        t2 = -2.0 * (q1 * q3 - q0 * q2)
        t3 = +2.0 * (q2 * q3 + q0 * q1)
        t4 = -2.0 * (q1 * q1 + q2sqr) + 1.0

        if t2 > 1.0:        t2 = 1.0
        if t2 < -1.0:       t2 = -1.0

        pitch = np.arcsin(t2)
        roll  = np.arctan2(t3, t4)
        yaw   = np.arctan2(t1, t0)
        pitch = 0
        roll = 0

        return yaw, pitch, roll
    
    
    def update_drone_relative_transform(self):
        # 1. compute relative transform in drone coords
        yaw, pitch, roll = self.yaw_pitch_roll()

        self.prev_trans = self.curr_trans.copy() 
        Rx = np.array([[1,0,0,0],
                       [0,np.cos(roll),-np.sin(roll),0, 1],
                       [0,np.sin(roll),np.cos(roll),0, 1],
                       [0, 0, 0, 1]])
        Ry = np.array([[np.cos(pitch),0,np.sin(pitch), 0],
                       [0,1,0,0],
                       [-np.sin(pitch),0,np.cos(pitch),0],
                       [0, 0, 0, 1]])
        Rz = np.array([[np.cos(yaw),-np.sin(yaw),0, 0],
                       [np.sin(yaw),np.cos(yaw),0, 0],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]])
        self.curr_trans = np.identity(4)
        self.curr_trans[:3, 3] = self.drone_pos
        self.curr_trans = np.matmul(Rz, self.curr_trans)

        curr_trans_inv = np.identity(4)
        curr_trans_inv[:3, :3] = self.curr_trans[:3, :3].T
        curr_trans_inv[:3, 3] = -np.matmul(self.curr_trans[:3, :3].T, self.curr_trans[:3, 3])

        # 2. to camera coordinates
        x_offset = np.pi
        y_offset = -np.pi*0.5
        Rx_offset = np.array([[1,0,0,0],
                              [0,np.cos(x_offset),-np.sin(x_offset),0],
                              [0,np.sin(x_offset),np.cos(x_offset),0],
                              [0, 0, 0, 1]])
        Ry_offset = np.array([[np.cos(y_offset),0,np.sin(y_offset), 0],
                              [0,1,0,0],
                              [-np.sin(y_offset),0,np.cos(y_offset),0],
                              [0, 0, 0, 1]])
        #self.dT = np.matmul(self.prev_trans , curr_trans_inv)  #drone rel transformation t-1 : t
        self.dT = np.matmul(Ry_offset, np.matmul(Rx_offset, np.matmul(self.prev_trans , np.linalg.inv(self.curr_trans))))

    def compute_u(self):
        #Input: (self.tracklets)
        #self.u1        N*(x, y): 2D top left corners of bounding boxes (tracklet state)
        #self.dT          (4, 4): Relative transformation
        #self.K           (4, 4): Intrinsics
        #self.XYZ N*(x, y, z, 1): Homogeneous 3D centers of objects

        #Output: 
        #self.u2     N*(x, y): 

        self.update_drone_relative_transform()
        u = {}
        #print(self.dT[:3, 3].ravel())

        # align indices
        for tr in self.tracklets:
            x, y, w, h = tr.getState() #xywh
            x_c = x + 0.5*w
            y_c = y + 0.5*h

            if tr.idx in self.XYZ.keys():
                pt = self.XYZ[tr.idx] #relative position

                u1 = np.matmul(np.matmul(self.K, self.prev_trans), pt)# K(H2*H1^-1)P
                u1 /= u1[-1]
                u2 = np.matmul(np.matmul(self.K, self.curr_trans), pt)# K(H2*H1^-1)P
                u2 /= u2[-1]

                du = u2 - u1
                print(-du[:2])
                u[tr.idx] = -du[:2] #

        self.u = u
        #print(self.u)

    def imageCb(self, data):
        try:
            #convert to opencv
            cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
        except CvBridgeError as e:
            print(e)

	    # Store raw detections
        if not self.offline:
            self.bboxes = self.yolo.detect(cv_image)
        
        if self.publish_raw_bboxes:
            raw_bboxes = BoundingBoxes()
            raw_bboxes.header = data.header
            raw_bboxes.image_header = data.header

            for [x, y, w, h, c] in self.bboxes:
                bbox_msg = BoundingBox()
                bbox_msg.xmin = x
                bbox_msg.ymin = y
                bbox_msg.xmax = x + w
                bbox_msg.ymax = y + h
                bbox_msg.label = c
                bbox_msg.idx = -1
                raw_bboxes.bounding_boxes.append(bbox_msg)

	        self.raw_pub.publish(raw_bboxes)

        # Store tracking results (to publish)
        bboxes = BoundingBoxes()
        bboxes.header = data.header
        bboxes.image_header = data.header
        
        self.compute_u()
        
        for tr in self.tracklets:
            #tr.predict(self.u[tr.idx]) # incorporate control inputs to existing tracklets, assign by idx
            tr.predict()
            tr.setActive(False)

        for yolo_box in self.bboxes:
            
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
                bbox_msg.idx = tr.idx
                bboxes.bounding_boxes.append(bbox_msg)

                if tr.timeout > 2:
                    self.tracklets.remove(tr)

        self.pub.publish(bboxes)
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
                    #color = self.colors[msg.bounding_boxes[i].idx]
                    color = [255, 0, 0]

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
