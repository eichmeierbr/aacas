#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Joy
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped, PointStamped, Point, Vector3, Quaternion
from scipy.spatial.transform import Rotation as R
from dji_m600_sim.srv import SimDroneTaskControl
from dji_sdk.srv import DroneTaskControl, SDKControlAuthority, SetLocalPosRef
from aacas_detection.srv import QueryDetections
from lidar_process.msg import tracked_obj, tracked_obj_arr



class Objects:
    def __init__(self, pos = np.zeros(3), vel=np.zeros(3), dist = np.inf, id=0):
        self.id = id
        self.position = pos
        self.velocity = vel
        self.distance = dist
  

class vectFieldController:

    def __init__(self, waypoints = [[0,0,0]]):
        self.v_max =  rospy.get_param('maximum_velocity')
        self.detections = []

        # Waypoint params
        self.waypoints = waypoints
        self.goalPt = 0
        self.goal = self.waypoints[self.goalPt]
        self.switch_dist =  rospy.get_param('switch_waypoint_distance')

        # Orbit params
        self.freq = -1 # Orbit direction (+: CW, -: ccw)
        self.safe_dist = rospy.get_param('safe_distance')
        self.rad = self.safe_dist # Radius of orbit
        self.k_conv =  rospy.get_param('orbit_k_conv') # Gain to converge to orbit
        self.K_theta =  rospy.get_param('heading_k_theta')
        self.last_orbit_change_ = rospy.Time.now()
        self.change_orbit_wait_ = rospy.get_param('orbit_change_wait')

        # Go to Goal Parameters
        self.g2g_sig =  rospy.get_param('g2g_sigma')
        self.g2g_sig_sq = self.g2g_sig**2

        self.in_avoid = False

        # state Information
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.yaw = 0
        self.yaw_rate = 0
        self.quat = Quaternion()
        self.pos_pt = Point()

        # Publisher Information
        vel_ctrl_pub_name = rospy.get_param('vel_ctrl_sub_name')
        self.vel_ctrl_pub_ = rospy.Publisher(vel_ctrl_pub_name, Joy, queue_size=10)

        pos_ctrl_pub_name = rospy.get_param('pos_ctrl_sub_name')
        self.pos_ctrl_pub_ = rospy.Publisher(pos_ctrl_pub_name, Joy, queue_size=10)

        # Subscriber Information
        rospy.Subscriber(rospy.get_param('position_pub_name'), PointStamped,      self.position_callback, queue_size=1)
        rospy.Subscriber(rospy.get_param('velocity_pub_name'), Vector3Stamped,    self.velocity_callback, queue_size=1)
        rospy.Subscriber(rospy.get_param('attitude_pub_name'), QuaternionStamped, self.attitude_callback, queue_size=1)

        # Service Information
        self.live_flight = rospy.get_param('live_flight',default=False)
        if self.live_flight:
            query_detections_name = rospy.get_param('query_detections_service')
            rospy.wait_for_service(query_detections_name)
            self.query_detections_service_ = rospy.ServiceProxy(query_detections_name, QueryDetections)
    
            takeoff_service_name = rospy.get_param('takeoff_land_service_name')
            rospy.wait_for_service(takeoff_service_name)
            self.takeoff_service = rospy.ServiceProxy(takeoff_service_name, DroneTaskControl)
    
            authority_service_name = 'dji_sdk/sdk_control_authority'
            rospy.wait_for_service(authority_service_name)
            self.authority_service = rospy.ServiceProxy(authority_service_name, SDKControlAuthority)

            set_pos_name = 'dji_sdk/set_local_pos_ref'
            rospy.wait_for_service(set_pos_name)
            self.set_pos_service = rospy.ServiceProxy(set_pos_name, SetLocalPosRef)
        else:
            query_detections_name = rospy.get_param('query_detections_service')
            rospy.wait_for_service(query_detections_name)
            self.query_detections_service_ = rospy.ServiceProxy(query_detections_name, QueryDetections)
    
            takeoff_service_name = rospy.get_param('takeoff_land_service_name')
            rospy.wait_for_service(takeoff_service_name)
            self.takeoff_service = rospy.ServiceProxy(takeoff_service_name, DroneTaskControl)


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
        r = R.from_quat([q.x, q.y, q.z, q.w])
        [roll, pitch, yaw] = r.as_euler('xyz')
        self.yaw = yaw

    ## TODO: Implement in 3D
    ## For Now: 2D Implementation
    def getXdes(self):
        velDes = np.zeros(4)

        if len(self.waypoints) == 0: return velDes
        
        # Check if we are close to an object
        [closeObject, move] = self.getCloseObject()

        # If object is close to the goal
        pos = closeObject.position
        # d = np.linalg.norm([pos.x - self.goal[0], pos.y - self.goal[1], pos.z - self.goal[2]])
        # v = [closeObject.velocity.x, closeObject.velocity.y, closeObject.velocity.z]
        # if all([move, d < self.safe_dist, np.linalg.norm(v) < 0.5]):
        #     velDes[:3] = self.repulsionField(closeObject)
        
        # If close to object, orbit
        if all([move, closeObject.distance < self.safe_dist]):
        # if all([closeObject.distance < self.safe_dist]):
            ## Orbit if the object comes into our safety radius, and is moving fast enough
            if self.change_orbit_wait_ < (rospy.Time.now() - self.last_orbit_change_).to_sec():
                self.decideOrbitDirection(closeObject)
            velDes[:3] = self.getOrbit([closeObject.position.x,closeObject.position.y,closeObject.position.z])
            self.in_avoid = True  

        # elif all([move, np.linalg.norm(v) < 0.5]):
        #     ## Orbit if the object comes into our safety radius, and is moving fast enough
        #     # if self.change_orbit_wait_ < (rospy.Time.now() - self.last_orbit_change_).to_sec():
        #     #     self.decideOrbitDirection(closeObject)
        #     safeD_copy = self.safe_dist
        #     self.safe_dist = 2
        #     velDes[:3] = self.getOrbit([closeObject.position.x,closeObject.position.y,closeObject.position.z])
        #     self.safe_dist = safeD_copy
        #     self.in_avoid = True  


        else: # Go to goal 
            velDes[:3] = self.goToGoalField()
            self.in_avoid = False

        # Normalize velocity
        if np.linalg.norm(velDes[:3]) > self.v_max:
            velDes[:3] = velDes[:3]/np.linalg.norm(velDes[:3])*self.v_max
        
        # Heading Control
        w_d = self.headingControl(velDes)
        velDes[3] = w_d

        return velDes

    def move(self):
        # Check if we have reached the next waypoint. If so, update
        self.changeGoalPt()
        self.v_max =  rospy.get_param('maximum_velocity')
        
        # Update Detections
        self.updateDetections()

        # Get velocity vector
        velDes = self.getXdes() 
        
        # Publish Vector
        joy_out = Joy()
        joy_out.header.stamp = rospy.Time.now()
        joy_out.axes = [velDes[0], velDes[1], velDes[2],velDes[3],]
        self.vel_ctrl_pub_.publish(joy_out)


    ## TODO: Implement in 3D
    ## For Now: 2D Implementation
    def getCloseObject(self):
        closeObject = Objects()
        closeObject.distance = np.inf
        move = False

        # Perform transformed coordinates
        T_vo = self.transformToGoalCoords()    
        
        for obst in self.detections:
            obs_pos = np.array([obst.position.x, obst.position.y, obst.position.z])
            obs_vel = np.array([obst.velocity.x, obst.velocity.y, obst.velocity.z])
            
            pos = np.array([obs_pos[0], obs_pos[1], 1])
            vel = np.array([obs_vel[0], obs_vel[1], 0])
            obst_trans = np.matmul(T_vo, pos)
            obst_vel   = np.matmul(T_vo, vel)
            other = np.dot(obst_vel, obst_trans)
            if all([obst.distance < closeObject.distance, obst_trans[1] > 0, other <= 1]):
                closeObject = obst
                move = True
        return [closeObject, move]


    def changeGoalPt(self):
        dist_to_goal = np.linalg.norm(self.pos-self.goal)

        if(dist_to_goal < self.switch_dist):
            self.goalPt += 1
            if(self.goalPt > len(self.waypoints)-1):
                self.goalPt = 0
            self.goal =self.waypoints[self.goalPt]


    def headingControl(self, velDes):
        # if self.in_avoid:
        #     vel_angle = np.arctan2(velDes[1], velDes[0])
        # else:
        #     vel_angle = - np.pi/2
        vel_angle = np.arctan2(velDes[1], velDes[0])

        angleDiff = vel_angle - self.yaw
        angleDiff = (angleDiff + np.pi) % (2 * np.pi) - np.pi
        w_d = self.K_theta * angleDiff

        # w_d = vel_angle
        return w_d


    def decideOrbitDirection(self, ob):
        # Note: All directions are assuming the vehicle is looking
        # straight at the goal

        obst_vel = np.array([ob.velocity.x, ob.velocity.y, ob.velocity.z])
        obst_pos = np.array([ob.position.x, ob.position.y, ob.position.z])
        
        
        # Perform transformed coordinates
        T_vo = self.transformToGoalCoords()
        
        trans_vel = np.matmul(T_vo, [obst_vel[0], obst_vel[1], 0])
        trans_pos = np.matmul(T_vo, [obst_pos[0], obst_pos[1], 1])

        # Check if object is moving
        if np.linalg.norm(trans_vel) > 1.1:
            if(trans_vel[0] >= 0):          # If obstacle is moving right
                self.freq = 1               # Orbit CW
            else:                           # If obstacle is moving left
                self.freq = -1              # Orbit CCW

        # else object is stationary
        else:
            if(trans_pos[0] >= 0):  # If object is to the right
                self.freq = 1       # Orbit CW
            else:                   # If object is to the left
                self.freq = -1      # Orbit CCW
        self.last_orbit_change_ = rospy.Time.now()


    ## TODO: Implement in 3D
    ## For Now: 2D Implementation
    def getOrbit(self, center):
        xhat = self.pos[:2] - center[:2] # Change to orbit coords
        gam = self.k_conv*(self.rad**2 - np.matmul(xhat, xhat) ) # Convergence to orbit


        A = np.array([[gam, self.freq], [-self.freq, gam]]) # Modified harmonic oscillator
        g = np.matmul(A, xhat[:2])   #  Calculate nominal velocity
        
        # Scale the vector field
        v_g = np.linalg.norm(g)
        g = self.v_max/v_g * g 
        
        # Pad output with z-vel
        velDes = np.array([g[0], g[1], 0])

        return velDes



    def goToGoalField(self):
        g = self.goal - self.pos
        
        # Scale the magnitude of the resulting vector
        dist2goal = np.linalg.norm(g)
        v_g = self.v_max * (1- np.exp(-dist2goal**2/self.g2g_sig_sq))
        
        if dist2goal > 0: # Avoid dividing by zero
            velDes = v_g/dist2goal * g # Dividing by dist is dividing by the norm
        else:
            velDes = np.array([0, 0, 0])
        
        if v_g > self.v_max:
            g = self.v_max/v_g * g

        return velDes


    def repulsionField(self, obs):
        g = self.goal - [obs.position.x, obs.position.y, obs.position.z]
        
        # Scale the magnitude of the resulting vector
        dist2goal = np.linalg.norm(g)
        v_g = self.v_max * (1- np.exp(-dist2goal**2/self.g2g_sig_sq))
        
        if dist2goal > 0: # Avoid dividing by zero
            velDes = v_g/dist2goal * g # Dividing by dist is dividing by the norm
        else:
            velDes = np.array([0, 0, 0])
        
        if v_g > self.v_max:
            g = self.v_max/v_g * g

        return -velDes

    ## TODO: Implement in 3D
    ## For Now: 2D Implementation
    # Transform coordinates. Origin on drone. X points to goal
    def transformToGoalCoords(self):

        dp = self.goal - self.pos

        th = np.arctan2(dp[1],dp[0]) - np.pi/2
        th = np.arctan2(np.sin(th), np.cos(th))
        R_ov = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
        t_ov = self.pos[:2]

        tempVect = np.matmul((-1 * R_ov.T), t_ov)
        T_vo = np.array([[R_ov[0,0], R_ov[1,0], tempVect[0]], [R_ov[0,1], R_ov[1,1], tempVect[1]], [0, 0, 1]])

        return T_vo


    def updateDetections(self):
        in_detections = self.query_detections_service_(vehicle_position=self.pos_pt, attitude=self.quat)
        self.detections = []
        for obj in in_detections.detection_array.tracked_obj_arr:
            newObj = Objects()
            newObj.position = obj.point
            newObj.velocity = Point(0,0,0)
            newObj.id = obj.object_id
            newObj.distance = np.linalg.norm([obj.point.x - self.pos[0], obj.point.y - self.pos[1], obj.point.z - self.pos[2]])
            self.detections.append(newObj)
            # self.detections = in_detections.detection_array.tracked_obj_arr


if __name__ == '__main__': 
  try:
    rospy.init_node('vectFieldController')

    start_pos = np.array([0,0,rospy.get_param('transform_z_offset')])

    # Launch Node
    field = vectFieldController()
    
    x_waypoint = rospy.get_param('waypoint_x')
    y_waypoint = rospy.get_param('waypoint_y')
    z_waypoint = rospy.get_param('waypoint_z')
    field.waypoints = np.transpose(np.array([x_waypoint, y_waypoint, z_waypoint]))
    field.goal = field.waypoints[0] 

    rospy.sleep(2)

    rospy.loginfo("LAUNCH")

    ########### Takeoff Controll ###############
    if field.live_flight:
        resp = field.authority_service(1)
        resp = field.set_pos_service()
    resp1 = field.takeoff_service(4)
    ########### Takeoff Controll ###############

    rospy.sleep(10)

    startTime = rospy.Time.now()
    rate = rospy.Rate(10) # 10hz
    while (rospy.Time.now() - startTime).to_sec() < 200:
        field.move()
        rate.sleep()

    rospy.loginfo("LAND")

    ########### Takeoff Controll ###############
    resp1 = field.takeoff_service(6)
    if field.live_flight:
        resp1 = field.authority_service(0)
    ########### Takeoff Controll ###############

    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass