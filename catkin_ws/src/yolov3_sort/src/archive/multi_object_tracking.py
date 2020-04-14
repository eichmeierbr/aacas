# USAGE
# python multi_object_tracking.py --video videos/soccer_01.mp4 --tracker csrt

# import the necessary packages
from imutils.video import VideoStream
from imutils.video import FPS
import argparse
import imutils
import time
import cv2



# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", type=str, default="videos/los_angeles.mp4",
	help="path to input video file")
ap.add_argument("-t", "--tracker", type=str, default="csrt",
	help="OpenCV object tracker type")
args = vars(ap.parse_args())

# initialize a dictionary that maps strings to their corresponding
# OpenCV object tracker implementations
OPENCV_OBJECT_TRACKERS = {
	"csrt": cv2.TrackerCSRT_create,
	"kcf": cv2.TrackerKCF_create,
	"boosting": cv2.TrackerBoosting_create,
	"mil": cv2.TrackerMIL_create,
	"tld": cv2.TrackerTLD_create,
	"medianflow": cv2.TrackerMedianFlow_create,
	"mosse": cv2.TrackerMOSSE_create
}

# initialize OpenCV's special multi-object tracker
trackers = cv2.MultiTracker_create()

# if a video path was not supplied, grab the reference to the web cam
if not args.get("video", False):
	print("[INFO] starting video stream...")
	vs = VideoStream(src=0).start()
	time.sleep(1.0)

# otherwise, grab a reference to the video file
else:
	vs = cv2.VideoCapture(args["video"])

fps = None

frame_num = 0
fps = FPS().start()
# loop over frames from the video stream
while True:
	# grab the current frame, then handle if we are using a
	# VideoStream or VideoCapture object
	frame = vs.read()
	frame = frame[1] if args.get("video", False) else frame

	# check to see if we have reached the end of the stream
	if frame is None:
		break

	# resize the frame (so we can process it faster)
	width_frame = 400
	frame = imutils.resize(frame, width=width_frame)

	# grab the updated bounding box coordinates (if any) for each
	# object that is being tracked
	(success, boxes) = trackers.update(frame)

	# loop over the bounding boxes and draw then on the frame
	for box in boxes:
		(x, y, w, h) = [int(v) for v in box]
		cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

	fps.update()
	fps.stop()
	print("FPS", "{:.2f}".format(fps.fps()))

	# show the output frame
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF

	# if the 's' key is selected, we are going to "select" a bounding
	# box to track
	if frame_num ==0:
		# if box ~= other boxes: need to be implemented (IOU)
		box = (int(439*width_frame/600),int(170*width_frame/600),int(110*width_frame/600),int(105*width_frame/600))

		# create a new object tracker for the bounding box and add it
		# to our multi-object tracker
		tracker = OPENCV_OBJECT_TRACKERS[args["tracker"]]()
		trackers.add(tracker, frame, box)

	if frame_num ==64:
		# box = int((322,166,37,63)/600*width_frame)
		box = (int(322*width_frame/600),int(166*width_frame/600),int(37*width_frame/600),int(63*width_frame/600))


		# create a new object tracker for the bounding box and add it
		# to our multi-object tracker
		tracker = OPENCV_OBJECT_TRACKERS[args["tracker"]]()
		trackers.add(tracker, frame, box)

	if key == ord("s"):
		# select the bounding box of the object we want to track (make
		# sure you press ENTER or SPACE after selecting the ROI)
		box = cv2.selectROI("Frame", frame, fromCenter=False,
			showCrosshair=True)

		print(box)
		print("frame",frame_num)

		# create a new object tracker for the bounding box and add it
		# to our multi-object tracker
		tracker = OPENCV_OBJECT_TRACKERS[args["tracker"]]()
		trackers.add(tracker, frame, box)

	# if the `q` key was pressed, break from the loop
	elif key == ord("q"):
		break

	frame_num += 1

# if we are using a webcam, release the pointer
if not args.get("video", False):
	vs.stop()

# otherwise, release the file pointer
else:
	vs.release()

# close all windows
cv2.destroyAllWindows()
