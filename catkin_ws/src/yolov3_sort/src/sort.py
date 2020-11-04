import numpy as np
import cv2
import imutils
import os, sys, glob
from filterpy.kalman import KalmanFilter

class Tracklet():
    """
    Generate tracklet for object being tracked.

    idx:     Object count (unique, ascending).
    label:   YOLO class prediction.
    bbox:    Initialize tracklet state (x, y, w, h) with the 1st YOLO detection.
    csrt:    Use CSRT tracking.

    active:  Flag indicator of whether object is still present in frame.
    timeout: Frame counts since object has "left".
    color:   Random RGB tuple for bounding box.

    kf:   Kalman filter object.
    kf.x: Kalman state vector = [x, y, w, h, dx, dy, dw, dh, ddx, ddw].
    kf.z: Kalman measurement vector (YOLO) = [x, y, w, h].
    kf.u: Kalman control input vector (CSRT) = [x, y, w, h].

    """
    def __init__(self, idx, label, bbox, csrt=False):
        self.idx = idx
        self.length = 0
        self.label = label
        self.csrt = csrt

        self.active = True
        self.timeout = 0
        self.color = (np.random.randint(255), np.random.randint(255), np.random.randint(255))

        dt = 1.0 / 30.0 #30fps
        self.kf = KalmanFilter(dim_x=10, dim_z=4)
        self.kf.x = np.array([bbox[0], bbox[1], bbox[2], bbox[3], 0., 0., 0., 0., 0., 0.])[np.newaxis].T

        self.kf.F = np.array([[1,0,0,0, dt,  0,  0,  0, 0.5*dt**2,         0],
                              [0,1,0,0,  0, dt,  0,  0,         0, 0.5*dt**2],
                              [0,0,1,0,  0,  0, dt,  0,         0,         0],
                              [0,0,0,1,  0,  0,  0, dt,         0,         0],
                              [0,0,0,0,  1,  0,  0,  0,        dt,         0],
                              [0,0,0,0,  0,  1,  0,  0,         0,        dt],
                              [0,0,0,0,  0,  0,  1,  0,         0,         0],
                              [0,0,0,0,  0,  0,  0,  1,         0,         0],
                              [0,0,0,0,  0,  0,  0,  0,         1,         0],
                              [0,0,0,0,  0,  0,  0,  0,         0,         1]],dtype=float)
        self.kf.H = np.array([[1,0,0,0,0,0,0,0,0,0],
                              [0,1,0,0,0,0,0,0,0,0],
                              [0,0,1,0,0,0,0,0,0,0],
                              [0,0,0,1,0,0,0,0,0,0]],dtype=float)
        #self.kf.B = np.identity(10)
        #self.kf.B[0:3,0:3] = np.array([[574.0198, 0.0, 318.1983],[0.0, 575.2453, 246.5657],[0.0, 0.0, 1.0]])
        self.kf.B = np.array([[dt, 0, 0, 0, 1, 0, 0, 0, 0, 0], 
                              [0, dt, 0, 0, 0, 1, 0, 0, 0, 0]]).T


        if self.csrt:
            for i in range(4):
                self.kf.F[i][i] = 0.
            self.kf.B = (self.kf.H).T

        self.kf.R[2:,2:] *= 0.001 #1
        self.kf.P[4:,4:] *= 1000. #1000 give high uncertainty to the unobservable initial velocities
        self.kf.P *= 10. #10

        self.kf.Q *= 10
        self.kf.Q[4:,4:] *= 100 #1000
        self.kf.Q[8:,8:] *= 100

    def predict(self, u, csrt_det=None):
        """
        Makes prediction of the current state.
        If using CSRT, predict state with CSRT bounding box as control input.
        If otherwise, predict state only using current state estimation.

        Input:  csrt_det = np.array([x, y, w, h])
        Output: None
        """
        if self.csrt:
            u = np.array(csrt_det)[np.newaxis].T
            self.kf.predict(u)
        else:
            large_u = np.identity(10)
            large_u[0:4,0:4] = u
            self.kf.predict(large_u)

    def update(self, yolo_det):
        """
        Updates current Kalman state using YOLO bounding box.

        Input:  yolo_det = np.array([x, y, w, h])
        Output: None
        """
        bbox = yolo_det[np.newaxis].T
        self.kf.update(bbox)

    def getState(self):
        """
        Returns current Kalman state.

        Input:  None
        Output: state = np.array([x, y, w, h])
        """
        return self.kf.x[:4].squeeze()

    def setActive(self, st):
        """
        set flag indicator for whether tracklet is still active.

        Input:  st (bool)
        Output: None
        """
        self.active = st

    def addLength(self):
        self.length += 1

    def addTimeout(self):
        """
        Increment frame count for tracklet being inactive (not detected).
        """
        self.timeout += 1

    def zeroTimeout(self):
        """
        Set inactive frame count to 0 (i.e. Object is detected again).
        """
        self.timeout = 0

def iou(p, y, th=0.1):
    """
    Compute IoU between prediction and detection, in order to associate detection to tracklet.

    Input:  p (prediction) = np.array([x, y, w, h])
            y (detection)  = np.array([x, y, w, h])
            th (threshold)
    Output: IoU > th (bool)
    """
    #pad reception ranges for small targets
    padx = 100. / ((p[2]+y[2])*0.5)
    pady = 100. / ((p[3]+y[3])*0.5)

    bb_pred = np.array([p[0] - padx, p[1] - pady, p[0] + p[2] + padx, p[1] + p[3] + pady])
    bb_meas = np.array([y[0] - padx, y[1] - pady, y[0] + y[2] + padx, y[1] + y[3] + pady])

    xx1 = np.maximum(bb_pred[0], bb_meas[0])
    yy1 = np.maximum(bb_pred[1], bb_meas[1])
    xx2 = np.minimum(bb_pred[2], bb_meas[2])
    yy2 = np.minimum(bb_pred[3], bb_meas[3])
    w = np.maximum(0., xx2 - xx1)
    h = np.maximum(0., yy2 - yy1)
    wh = w * h
    o = wh / ((bb_pred[2]-bb_pred[0])*(bb_pred[3]-bb_pred[1])
              + (bb_meas[2]-bb_meas[0])*(bb_meas[3]-bb_meas[1]) - wh)

    return(o > th)

