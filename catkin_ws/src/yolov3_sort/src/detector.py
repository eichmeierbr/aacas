from __future__ import division

from models import *
from utils.utils import *
from utils.datasets import *

import numpy as np
import os
import sys

import cv2
import torch

class Detector():
    """
    Detector: YOLO model object.

    model_def:    Model architecture.
    weights_path: Path to custom .pth weights file.
    classes:      List of class labels.
    conf_thres:   Threshold for objectness confidence score.
    nms_thres:    Threshold for applying non-max suppression.
    """
    def __init__(self, model_def, weights_path, class_path, conf_thres, nms_thres, img_size):
        self.model_def = model_def
        self.weights_path = weights_path
        self.class_path = class_path
        self.conf_thres = conf_thres
        self.nms_thres = nms_thres
        self.img_size = img_size

        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = Darknet(self.model_def, img_size=self.img_size).to(device)
        if torch.cuda.is_available():
            self.model.load_state_dict(torch.load(self.weights_path))
        else:
            self.model.load_state_dict(torch.load(self.weights_path, map_location=lambda storage, loc: storage))

        self.model.eval()  # Set in evaluation mode
        self.classes = load_classes(self.class_path)  # Extracts class labels from file
        self.Tensor = torch.cuda.FloatTensor if torch.cuda.is_available() else torch.FloatTensor

    def detect(self, frame): #frame is a numpy array
        """
        Generate object detections and class predictions from frame.

        Input:  frame = np.array([H, W, 3])
        Output: bboxes (contains multiple detections)
                       = np.array([[x, y, w, h, label],
                                   [x, y, w, h, label], ...])
        """
        x = torch.from_numpy(frame.transpose(2, 0, 1))
        x = x.unsqueeze(0).float()

        _, _, h, w = x.size()
        ih, iw = (416, 416)
        dim_diff = np.abs(h - w)
        pad1, pad2 = int(dim_diff // 2), int(dim_diff - dim_diff // 2)
        pad = (pad1, pad2, 0, 0) if w <= h else (0, 0, pad1, pad2)
        x = F.pad(x, pad=pad, mode='constant', value=127.5) / 255.0
        x = F.upsample(x, size=(ih, iw), mode='bilinear')

        img_detections = []
        with torch.no_grad():
            if torch.cuda.is_available():
                x = x.cuda()
            detections = self.model(x)
            detections = non_max_suppression(detections, self.conf_thres, self.nms_thres)
            img_detections.extend(detections)

        for detections in img_detections:
            bboxes = []
            if detections is not None:
                # Rescale boxes to original image
                dets = rescale_boxes(detections, self.img_size, frame.shape[:2]).cpu()
                #unique_labels = detections[:, -1].cpu().unique()
                #n_cls_preds = len(unique_labels)
                for x1, y1, x2, y2, conf, cls_conf, cls_pred in dets:
                    bboxes.append([x1, y1, x2-x1, y2-y1, cls_pred])

        return np.array(bboxes)

