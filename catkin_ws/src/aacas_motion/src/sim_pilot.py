#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32, UInt8
from sensor_msgs.msg import Joy
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped, PointStamped, Point, Vector3, Quaternion
from dji_m600_sim.srv import SimDroneTaskControl
from traj_prediction.msg import tracked_obj, tracked_obj_arr



class Objects:
    def __init__(self, pos = np.zeros(3), vel=np.zeros(3), dist = np.inf, id=0):
        self.id = id
        self.position = pos
        self.velocity = vel
        self.distance = dist
        self.last_orbit_change_ = rospy.Time.now() - rospy.Duration(secs=1000)
        self.orbit = -1

  

class simPilot:

    def __init__(self, waypoints = [[0,0,0]]):
        self.v_max =  rospy.get_param('maximum_velocity')
        self.detections = []

        # Waypoint params
        self.waypoints = waypoints
        #self.h_max = np.max(np.array(self.waypoints)[:, 2]) + 0.5
        self.goalPt = 0
        self.goal = self.waypoints[self.goalPt]
        self.switch_dist =  rospy.get_param('switch_waypoint_distance')
        self.last_waypoint_time = rospy.Time.now()
        self.waypoint_wait_time = rospy.get_param('waypoint_wait', default=5.0)

        # state Information
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.yaw = 0
        self.yaw_rate = 0
        self.quat = Quaternion()
        self.pos_pt = Point()
        self.is_safe = True

        # safety check constraints
        self.x_constraint = rospy.get_param('x_constraint')
        self.y_constraint = rospy.get_param('y_constraint')
        self.is_ready = False # Before reaching 1st waypoint

        self.angular_vel = np.zeros(3)
        self.acceleration = np.zeros(3)


        # Publisher Information
        vel_ctrl_pub_name = rospy.get_param('vel_ctrl_sub_name')
        self.vel_ctrl_pub_ = rospy.Publisher(vel_ctrl_pub_name, Joy, queue_size=10)

        pos_ctrl_pub_name = rospy.get_param('pos_ctrl_sub_name')
        self.pos_ctrl_pub_ = rospy.Publisher(pos_ctrl_pub_name, Joy, queue_size=10)

        self.goal_pub_ = rospy.Publisher('current_goal', Point, queue_size=10)
        

        # Subscriber Information
        rospy.Subscriber(rospy.get_param('position_pub_name'), PointStamped,      self.position_callback, queue_size=1)
        rospy.Subscriber(rospy.get_param('velocity_pub_name'), Vector3Stamped,    self.velocity_callback, queue_size=1)
        rospy.Subscriber(rospy.get_param('attitude_pub_name'), QuaternionStamped, self.attitude_callback, queue_size=1)
        rospy.Subscriber('aacas_velocity', Joy, self.command_vel_callback, queue_size=1)
        # rospy.Subscriber('/height_above_takeoff', Float32, self.height_above_takeoff_cb)


        tracked_obj_topic = rospy.get_param('obstacle_trajectory_topic')
        rospy.Subscriber(tracked_obj_topic, tracked_obj_arr, self.updateDetections, queue_size=1)

        # Service Information
        # rospy.Subscriber('/dji_sdk/angular_velocity_fused', Vector3Stamped, self.angular_vel_cb)
        # rospy.Subscriber('/dji_sdk/acceleration_ground_fused', Vector3Stamped, self.acceleration_cb)

        takeoff_service_name = rospy.get_param('takeoff_land_service_name')
        rospy.wait_for_service(takeoff_service_name)
        self.takeoff_service = rospy.ServiceProxy(takeoff_service_name, SimDroneTaskControl)



    def angular_vel_cb(self, msg):
        vec = msg.vector
        self.angular_vel = np.array([vec.x, vec.y, vec.z])

    def acceleration_cb(self, msg):
        vec = msg.vector
        self.acceleration = np.array([vec.x, vec.y, vec.z])   

    def position_callback(self, msg):
        pt = msg.point
        self.pos_pt = pt
        self.pos = np.array([pt.x, pt.y, pt.z])

    def velocity_callback(self, msg):
        pt = msg.vector
        self.vel = np.array([pt.x, pt.y, pt.z])

    def attitude_callback(self, msg):
        q = msg.quaternion
        self.quat = q

        qt = [q.w, q.x, q.y, q.z]
        [yaw, pitch, roll] = self.yaw_pitch_roll(qt)
        self.yaw = yaw


    def yaw_pitch_roll(self, q):
        q0, q1, q2, q3 = q
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

        return yaw, pitch, roll  

    def command_vel_callback(self, msg):
        self.vel_ctrl_pub_.publish(msg)


    def updateDetections(self, msg):
        # in_detections = self.query_detections_service_(vehicle_position=self.pos_pt, attitude=self.quat)
        in_detections = msg.tracked_obj_arr
        self.detections = []
        for obj in in_detections:
            newObj = Objects()
            newObj.position = obj.point
            newObj.velocity = Point(0,0,0)
            newObj.id = obj.object_id
            newObj.distance = np.linalg.norm([obj.point.x - self.pos[0], obj.point.y - self.pos[1], obj.point.z - self.pos[2]])
            self.detections.append(newObj)
            # self.detections = in_detections.detection_array.tracked_obj_arr


    def hoverInPlace(self):
        # Safety hovering
        # Publish Vector, stay in place
        joy_out = Joy()
        joy_out.header.stamp = rospy.Time.now()
        joy_out.axes = [0.0, 0.0, 0.0, 0.0]
        self.vel_ctrl_pub_.publish(joy_out)
        

    def rush(self):
        # Safety violation test, rush in x direction
        # Publish Vector
        joy_out = Joy()
        joy_out.header.stamp = rospy.Time.now()
        joy_out.axes = [self.v_max * 1.5, 0., 0., 0.]
        self.vel_ctrl_pub_.publish(joy_out)


if __name__ == '__main__': 
  try:
    rospy.init_node('sim_pilot')
    rate = rospy.Rate(10) # 10hz

    # Launch Node
    pilot = simPilot()

    x_waypoint = rospy.get_param('waypoint_x')
    y_waypoint = rospy.get_param('waypoint_y')
    z_waypoint = rospy.get_param('waypoint_z')
    pilot.waypoints = np.transpose(np.array([x_waypoint, y_waypoint, z_waypoint]))
    pilot.goal = pilot.waypoints[0] 

    rospy.sleep(2)


    ########### Takeoff Controll ###############
    rospy.loginfo("LAUNCH")
    resp1 = pilot.takeoff_service(4)

    rospy.sleep(5)

    startTime = rospy.Time.now()
    while (rospy.Time.now() - startTime).to_sec() < 200 and pilot.is_safe:
        # if ((rospy.Time.now() - startTime).to_sec() >50): 
            # pilot.rush()
        # else: pilot.move()
        pilot.is_safe = True
        rate.sleep()
    
    if pilot.is_safe: # If the planner exited normally, land
        rospy.loginfo("LAND")
    
        ########### Landing Controll ###############
        resp1 = pilot.takeoff_service(6)
        ########### Landing Controll ###############

    else: # The planner detected unsafe conditions
        rospy.logerr("Unsafe condition detected. Hover for ten seconds.")
        ##
        ## Error protocol goes here
        pilot.hoverInPlace()
        rospy.sleep(10)
        rospy.logerr("Unsafe condition detected. Land and relaunch to restart.")
        resp1 = pilot.takeoff_service(6)
        ##


    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass

