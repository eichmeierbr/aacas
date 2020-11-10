#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32, UInt8, Bool
from sensor_msgs.msg import Joy, NavSatFix, BatteryState, Imu
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped, PointStamped, Point, Vector3, Quaternion
from dji_sdk.srv import DroneTaskControl, SDKControlAuthority, SetLocalPosRef
from traj_prediction.msg import tracked_obj, tracked_obj_arr



class Objects:
    def __init__(self, pos = np.zeros(3), vel=np.zeros(3), dist = np.inf, id=0):
        self.id = id
        self.position = pos
        self.velocity = vel
        self.distance = dist
        self.last_orbit_change_ = rospy.Time.now() - rospy.Duration(secs=1000)
        self.orbit = -1

  

class djiPilot:

    def __init__(self, waypoints = [[0,0,0]]):
        self.detections = []

        # Waypoint params
        self.waypoints = waypoints
        self.goalPt = 0
        self.goal = self.waypoints[self.goalPt]

        # state Information
        self.pos = np.zeros(3)
        self.yaw = 0
        self.pitch=0
        self.roll =0
        self.yaw_rate = 0
        self.quat = Quaternion()
        self.pos_pt = Point()
        self.can_command = False

        self.gps_health = 0
        self.min_gps    = rospy.get_param('min_gps_health', default=4)
        rospy.Subscriber('/dji_sdk/gps_health', UInt8, self.gps_health_cb)

        self.is_safe = False
        self.safe_time = rospy.Time.now()

        # Publisher Information
        vel_ctrl_pub_name = rospy.get_param('vel_ctrl_sub_name')
        self.vel_ctrl_pub_ = rospy.Publisher(vel_ctrl_pub_name, Joy, queue_size=10)

        pos_ctrl_pub_name = rospy.get_param('pos_ctrl_sub_name')
        self.pos_ctrl_pub_ = rospy.Publisher(pos_ctrl_pub_name, Joy, queue_size=10)

        # Subscriber Information
        rospy.Subscriber('aacas_velocity', Joy, self.command_vel_callback, queue_size=1)

        tracked_obj_topic = rospy.get_param('obstacle_trajectory_topic')
        rospy.Subscriber(tracked_obj_topic, tracked_obj_arr, self.updateDetections, queue_size=1)

        rospy.Subscriber(rospy.get_param('position_pub_name'), PointStamped,      self.position_callback, queue_size=1)
        rospy.Subscriber(rospy.get_param('safety_topic_name',default='safety_status'), Bool, self.safety_cb, queue_size=1)


        # Service Information
        takeoff_service_name = rospy.get_param('takeoff_land_service_name')
        rospy.wait_for_service(takeoff_service_name)
        self.takeoff_service = rospy.ServiceProxy(takeoff_service_name, DroneTaskControl)

        authority_service_name = 'dji_sdk/sdk_control_authority'
        rospy.wait_for_service(authority_service_name)
        self.authority_service = rospy.ServiceProxy(authority_service_name, SDKControlAuthority)

        set_pos_name = 'dji_sdk/set_local_pos_ref'
        rospy.wait_for_service(set_pos_name)
        self.set_pos_service = rospy.ServiceProxy(set_pos_name, SetLocalPosRef)


    def position_callback(self, msg):
        pt = msg.point
        self.pos_pt = pt
        self.pos = np.array([pt.x, pt.y, pt.z])
        self.last_pos = rospy.Time.now()


    def gps_health_cb(self, msg):
        self.gps_health = msg.data


    def command_vel_callback(self, msg):
        if self.is_safe and self.can_command:
            self.vel_ctrl_pub_.publish(msg)
        else:
            self.hoverInPlace()



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





    def safety_cb(self, msg):
        self.is_safe = msg.data
        self.safe_time = rospy.Time.now()



    def hoverInPlace(self):
        # Safety hovering
        # Publish Vector, stay in place
        joy_out = Joy()
        joy_out.header.stamp = rospy.Time.now()
        joy_out.axes = [0.0, 0.0, 0.0, 0.0]
        self.vel_ctrl_pub_.publish(joy_out)
        


if __name__ == '__main__': 
  try:
    rospy.init_node('dji_pilot')
    rate = rospy.Rate(10) # 10hz

    # Launch Node
    pilot = djiPilot()

    x_waypoint = rospy.get_param('waypoint_x')
    y_waypoint = rospy.get_param('waypoint_y')
    z_waypoint = rospy.get_param('waypoint_z')
    pilot.waypoints = np.transpose(np.array([x_waypoint, y_waypoint, z_waypoint]))
    pilot.goal = pilot.waypoints[0] 


    ########### Wait for GPS Lock  #############
    ## TODO: Add functionality to check number of satellites
    rospy.loginfo("Getting Satellite Fix")
    while pilot.gps_health < pilot.min_gps:
        rate.sleep()
    rospy.loginfo("Obtained Satellite Fix")

    ########### Set Local Position ###############
    resp = False
    rospy.loginfo("Setting Local Position Reference")
    while resp == False:
        resp = pilot.set_pos_service()
        rate.sleep()
    rospy.loginfo("Local Position Reference Set")


    ########### Get Control Authority ###############
    resp = False
    rospy.loginfo("Requesting Control Authority")
    while resp == False:
        resp = pilot.authority_service(1)
        rate.sleep()
    rospy.loginfo("Control Authority Granted")
    
   
    rospy.loginfo('Waiting for safe flight conditions')
    pilot.is_safe = False
    while not pilot.is_safe:
        if pilot.is_safe == False:
            rospy.logwarn_throttle(2,'Unsafe conditions. No launch')
    rospy.loginfo('Safe Flight Condition Confirmed')

    ########### Takeoff Control ###############
    rospy.loginfo("LAUNCH")
    resp1 = pilot.takeoff_service(4)

    rospy.sleep(5)


    startTime = rospy.Time.now()
    pilot.can_command = True
    while (rospy.Time.now() - startTime).to_sec() < 200 and pilot.is_safe:
        rate.sleep()
    
    if pilot.is_safe: # If the planner exited normally, land
        rospy.loginfo("LANDING")
        resp1 = pilot.takeoff_service(6)

    ## Error Protocol ##
    else: # The planner detected unsafe conditions
        rospy.logerr("Unsafe condition detected. Hover for ten seconds.")
        pilot.hoverInPlace()
        rospy.sleep(10)
        rospy.logerr("Unsafe condition detected. Land and relaunch to restart.")
        resp1 = pilot.takeoff_service(6)
        ##

    resp1 = pilot.authority_service(0)


    rospy.spin()
    
  except rospy.ROSInterruptException:
        resp1 = pilot.takeoff_service(6)
        resp1 = pilot.authority_service(0)
    # pass

