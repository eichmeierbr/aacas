from __future__ import division

from models import *
from utils.utils import *
from utils.datasets import *

import numpy as np
import os
import sys
import time
import datetime
import argparse

import cv2
import torch

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--image_folder", type=str, default="/home/aacas/tracker_aacas/data/samples")
    parser.add_argument("--model_def", type=str, default="config/yolov3-custom.cfg", help="path to model definition file")
    parser.add_argument("--weights_path", type=str, default="weights/yolov3_v2.pth", help="path to weights file")
    parser.add_argument("--class_path", type=str, default="data/custom/classes.names", help="path to class label file")
    parser.add_argument("--conf_thres", type=float, default=0.8, help="object confidence threshold")
    parser.add_argument("--nms_thres", type=float, default=0.1, help="iou thresshold for non-maximum suppression")
    parser.add_argument("--batch_size", type=int, default=1, help="size of the batches")
    parser.add_argument("--n_cpu", type=int, default=0, help="number of cpu threads to use during batch generation")
    parser.add_argument("--img_size", type=int, default=416, help="size of each image dimension")
    opt = parser.parse_args()
    print(opt)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    os.makedirs("output", exist_ok=True)

    # Set up model
    model = Darknet(opt.model_def, img_size=opt.img_size).to(device)

    if opt.weights_path.endswith(".weights"):
        # Load darknet weights
        model.load_darknet_weights(opt.weights_path)
    else:
        # Load checkpoint weights
        model.load_state_dict(torch.load(opt.weights_path))

    model.eval()  # Set in evaluation mode
    classes = load_classes(opt.class_path)  # Extracts class labels from file
    Tensor = torch.cuda.FloatTensor if torch.cuda.is_available() else torch.FloatTensor


    #cap = cv2.VideoCapture(0)
    for i in range(0, 76):
        prev_time = time.time()
        # Capture frame-by-frame
        #ret, frame = cap.read()
        frame = cv2.imread(os.path.join(opt.image_folder, "frame{:04d}.jpg".format(i)))
        #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        x = torch.from_numpy(frame.transpose(2, 0, 1))
        x = x.unsqueeze(0).float()

        # Apply letterbox resize
        _, _, h, w = x.size()
        ih, iw = (416, 416)
        dim_diff = np.abs(h - w)
        pad1, pad2 = int(dim_diff // 2), int(dim_diff - dim_diff // 2)
        pad = (pad1, pad2, 0, 0) if w <= h else (0, 0, pad1, pad2)
        x = F.pad(x, pad=pad, mode='constant', value=127.5) / 255.0
        x = F.upsample(x, size=(ih, iw), mode='bilinear') # x = (1, 3, 416, 416)

        img_detections = []
        # Get detections
        with torch.no_grad():
            x = x.cuda()
            detections = model(x)
            detections = non_max_suppression(detections, opt.conf_thres, opt.nms_thres)

        # Log progress
        current_time = time.time()
        inference_time = datetime.timedelta(seconds=current_time - prev_time)
        prev_time = current_time
        #print("\t+ Batch %d, Inference Time: %s" % (0, inference_time))
        img_detections.extend(detections)

        # Draw bounding boxes and labels of detections
        for detections in img_detections:
            if detections is not None:
                # Rescale boxes to original image
                detections = rescale_boxes(detections, opt.img_size, frame.shape[:2])
                unique_labels = detections[:, -1].cpu().unique()
                n_cls_preds = len(unique_labels)
                for x1, y1, x2, y2, conf, cls_conf, cls_pred in detections:

                    #print("\t+ Label: %s, Conf: %.5f" % (classes[int(cls_pred)], cls_conf.item()))
                    #box_w = x2 - x1
                    #box_h = y2 - y1
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    #cv2.line(img=frame, pt1=(x1,y1), pt2=(x1,y2), color=(255, 0, 0), thickness=2, lineType=8, shift=0)
                    #cv2.line(img=frame, pt1=(x2,y1), pt2=(x2,y2), color=(255, 0, 0), thickness=2, lineType=8, shift=0)
                    #cv2.line(img=frame, pt1=(x1,y2), pt2=(x2,y2), color=(255, 0, 0), thickness=2, lineType=8, shift=0)
                    #cv2.line(img=frame, pt1=(x1,y1), pt2=(x2,y1), color=(255, 0, 0), thickness=2, lineType=8, shift=0)

        # Display the resulting frame
        #frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        cv2.imshow('frame',frame)
        key = cv2.waitKey(1) & 0xFF

    #cap.release()
    cv2.destroyAllWindows()
