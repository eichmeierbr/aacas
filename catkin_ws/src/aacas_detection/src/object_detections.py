#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped, PointStamped, Point, Vector3, Quaternion
from lidar_process.msg import tracked_obj, tracked_obj_arr
from aacas_detection.srv import QueryDetections, QueryDetectionsResponse
import copy

class DetectionSimulation:
  def __init__(self):
    self.true_detections_ = []
    # self.sim_detections_ = ObstacleDetectionArray()
    # self.true_out_detections_ = ObstacleDetectionArray()

    self.sim_detections_ = tracked_obj_arr()
    self.true_out_detections_ = tracked_obj_arr()

    # Publisher for the detections
    detection_pub_name = rospy.get_param('true_obstacle_topic')
    self.detection_pub_ = rospy.Publisher(detection_pub_name, tracked_obj_arr, queue_size=1)

    # Service to report simulated detections
    detections_service_name = rospy.get_param('query_detections_service')
    self.task_ctrl_service_ = rospy.Service(detections_service_name, QueryDetections, self.reportDetections)

    


  def updateDetections(self, pos=np.zeros(3), quat=[0,0,0,1], true_detections=False):

    if true_detections: 
      # true_detects = ObstacleDetectionArray()
      true_detects = tracked_obj_arr()
      for obj in self.true_detections_:
        out_detection = obj.convertToDetectionMessage(orig_pos = pos)
        true_detects.tracked_obj_arr.append(out_detection)
      self.true_out_detections_ = true_detects

    else: 
      # sim_detections = ObstacleDetectionArray()
      sim_detections = tracked_obj_arr()
      for detection in self.true_detections_:
        obj = detection.fake_detection()
        obj.dist = np.linalg.norm(pos - np.array([obj.pos[0], obj.pos[1], obj.pos[2]]))

        if detection.detect_rate > np.random.uniform() and obj.dist < detection.detect_range:
          out_detection = obj.convertToDetectionMessage(orig_pos = pos)
          sim_detections.tracked_obj_arr.append(out_detection)

          self.sim_detections_ = sim_detections


  def publishDetections(self):
    self.updateDetections(true_detections=True)
    self.detection_pub_.publish(self.true_out_detections_)


  def reportDetections(self, req):
    veh_pos = [req.vehicle_position.x, req.vehicle_position.y, req.vehicle_position.z]
    self.updateDetections(pos=veh_pos, quat=req.attitude, true_detections=False)
    return self.sim_detections_




class Objects:
  def __init__(self, ident = 0, class_name=0, pos = np.zeros(3), vel=np.zeros(3), dist = np.inf):
    self.class_name = class_name
    self.pos = pos
    self.vel = [rospy.get_param('ob_vel_x'), rospy.get_param('ob_vel_y'), rospy.get_param('ob_vel_z')]
    self.dist = dist
    self.pos_noise = rospy.get_param('object_position_noise')
    self.vel_noise = rospy.get_param('object_velocity_noise')
    self.detect_rate = rospy.get_param('object_detection_rate')
    self.detect_range = rospy.get_param('object_detection_dist')
    self.id = ident
    self.lastUpdate = 0

  def get_position(self):
    return self.pos + np.random.randn(3) * self.pos_noise

  def get_velocity(self):
    return self.vel + np.random.randn(3) * self.vel_noise

  def fake_detection(self):
    outObj = copy.copy(self)
    outObj.pos = self.get_position()
    outObj.vel = self.get_velocity()
    return outObj
    
  def convertToDetectionMessage(self, orig_pos = [0,0,0]):
    dist = np.linalg.norm(orig_pos - np.array([self.pos[0], self.pos[1], self.pos[2]]))

    travel_dist = 50.0
    dy = rospy.get_param('ob_vel_y')
    dx = rospy.get_param('ob_vel_x')
    nowTime = rospy.Time.now().to_sec()
    dt = nowTime - self.lastUpdate
    self.lastUpdate = nowTime

    self.pos[1] += dy*dt
    if self.pos[1] > travel_dist/2: self.pos[1] = -travel_dist/2
    # self.pos[0] = dx * rospy.Time.now().to_sec() % travel_dist
    self.vel[0] = dx
    self.vel[1] = dy

    out_detection = tracked_obj()
    out_detection.object_id = self.id
    out_detection.object_label = self.class_name
    out_detection.point = Point(self.pos[0], self.pos[1], self.pos[2])

    # out_detection.velocity = Vector3(self.vel[0], self.vel[1], self.vel[2]) 
    # out_detection.distance = dist

    return out_detection
  


if __name__ == '__main__': 
  try:
    rospy.init_node('simulated_detector')

    detector = DetectionSimulation()

    obs_x = rospy.get_param('ob_start_x')
    obs_y = rospy.get_param('ob_start_y')
    obs_z = rospy.get_param('ob_start_z')

    for i in range(len(obs_x)):
      ob_start = np.array([obs_x[i], obs_y[i], obs_z[i]])
      obstacle = Objects(pos=ob_start, ident=i)
  
      detector.true_detections_.append(obstacle)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        detector.publishDetections()
        rate.sleep()

    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass

