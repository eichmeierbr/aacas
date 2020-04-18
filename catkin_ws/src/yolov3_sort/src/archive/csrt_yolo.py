import numpy as np
import cv2
import imutils
import os, sys, glob
from filterpy.kalman import KalmanFilter

class Tracklet():
    def __init__(self, idx, label, bbox):
        self.idx = idx
        self.label = label
        self.active = True
        self.timeout = 0
        self.color = (np.random.randint(255), np.random.randint(255), np.random.randint(255))

        dt = 1.0 / 30.0 #30fps
        self.kf = KalmanFilter(dim_x=10, dim_z=4)
        self.kf.x = np.array([bbox[0], bbox[1], bbox[2], bbox[3], 0., 0., 0., 0., 0., 0.])[np.newaxis].T
        # x = [x, y, w, h, dx, dy, dw, dh, ddx, ddy]
        # z = [x, y, w, h]
        # u = [x, y, w, h]

        self.kf.F = np.array([[0,0,0,0, dt,  0,  0,  0, 0.5*dt**2,         0],
                              [0,0,0,0,  0, dt,  0,  0,         0, 0.5*dt**2],
                              [0,0,0,0,  0,  0, dt,  0,         0,         0],
                              [0,0,0,0,  0,  0,  0, dt,         0,         0],
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
        self.kf.B = (self.kf.H).T

        self.kf.R[2:,2:] *= 0.001 #1
        self.kf.P[4:,4:] *= 1000. #1000 give high uncertainty to the unobservable initial velocities
        self.kf.P *= 10. #10

        self.kf.Q *= 10
        self.kf.Q[4:,4:] *= 100 #1000
        self.kf.Q[8:,8:] *= 100

    def predict(self, csrt_det):
        u = np.array(csrt_det)[np.newaxis].T
        self.kf.predict(u)

    def update(self, yolo_det):
        bbox = yolo_det[np.newaxis].T
        self.kf.update(bbox)

    def getState(self):
        return self.kf.x[:4].squeeze()

    def setActive(self, st):
        # flag indicator for whether tracklet is still active
        self.active = st

    def addTimeout(self):
        self.timeout += 1

    def zeroTimeout(self):
        self.timeout = 0

def iou(p, y, th=0.1):
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

if __name__ == "__main__":
    OPENCV_OBJECT_TRACKERS = {
    "csrt": cv2.TrackerCSRT_create,
    }
    trackers = cv2.MultiTracker_create()

    img_root = '/home/szuyu/Downloads/img'
    txt_root = '/home/szuyu/Downloads/output'

    tracklets = [] #list of active objects
    object_id = 0

    for i in range(1, 601):
        #open images and detections
        txt = open(os.path.join(txt_root, '{:06d}.txt'.format(i)))
        frame = cv2.imread(os.path.join(img_root, "{:06d}.jpg".format(i)))
        _, _W, _ = frame.shape
        frame = imutils.resize(frame, width=400)
        resize_ratio = 400. / _W

        yolo_boxes = []
        for ln in txt:
            st = ln.rstrip().split(",")
            x, y = float(st[0]), float(st[1])
            w, h = float(st[2]) - x, float(st[3]) - y
            yolo_box = np.array([x*resize_ratio, y*resize_ratio, w*resize_ratio, h*resize_ratio, int(st[-1])])
            yolo_boxes.append(yolo_box)

        for tr in tracklets:
            tr.setActive(False)

        for yolo_box in yolo_boxes:
            match = 0
            for tr in tracklets:
                if yolo_box[-1] != tr.label:
                    continue

                if iou(yolo_box[:4], tr.getState()):
                    tr.update(yolo_box[:4])
                    match = 1
                    tr.setActive(True)
                    break

            if match == 0:
                #add new tracklet
                tr = Tracklet(object_id, yolo_box[-1], yolo_box[:-1])
                tracklets.append(tr)
                tracker = OPENCV_OBJECT_TRACKERS["csrt"]()
                trackers.add(tracker, frame, (yolo_box[0], yolo_box[1], yolo_box[2], yolo_box[3]))
                object_id += 1

        for tr in tracklets:
            if tr.active==False:
                tr.addTimeout()
            if tr.timeout > 5:
                tracklets.remove(tr)
                continue

            if tr.active:
                (x, y, w, h) = tr.getState()
                x, y, w, h = int(x), int(y), int(w), int(h)
                cv2.rectangle(frame, (x, y), (x + w, y + h), tr.color, 2)

            (success, csrt_boxes) = trackers.update(frame)
            tr.predict(csrt_boxes[tr.idx])

        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF

cv2.destroyAllWindows()
