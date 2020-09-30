#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Joy
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped, PointStamped, Point, Vector3, Quaternion
from scipy.spatial.transform import Rotation as R
# from dji_m600_sim.srv import DroneTaskControl
from dji_sdk.srv import DroneTaskControl
from aacas_detection.srv import QueryDetections
from aacas_detection.msg import ObstacleDetection



class Objects:
    def __init__(self, pos = np.zeros(3), vel=np.zeros(3), dist = np.inf):
        self.pos = pos
        self.vel = vel
        self.dist = dist
  

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

        # Subscriber Information
        rospy.Subscriber(rospy.get_param('position_pub_name'), PointStamped,      self.position_callback, queue_size=1)
        rospy.Subscriber(rospy.get_param('velocity_pub_name'), Vector3Stamped,    self.velocity_callback, queue_size=1)
        rospy.Subscriber(rospy.get_param('attitude_pub_name'), QuaternionStamped, self.attitude_callback, queue_size=1)

        # Service Information
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
        closeObjects, avoid = self.getCloseObjects()
        
        # If close to object, orbit
        if avoid:
            vels = []
            for obstacle in closeObjects:
                # if self.change_orbit_wait_ < (rospy.Time.now() - self.last_orbit_change_).to_sec():
                #     self.decideOrbitDirection(obstacle)
                # velDes[:3] = self.getOrbit([obstacle.position.x,obstacle.position.y,obstacle.position.z])

                self.decideOrbitDirection(obstacle)
                vel = self.getOrbit([obstacle.position.x,obstacle.position.y,obstacle.position.z])
                mod = 1/(obstacle.distance)
                # mod = np.exp(-1/(3*obstacle.distance))
                vels.append(vel * mod)
            velDes[:3] = np.sum(vels,axis=0)
            velDes[:3] = velDes[:3]/np.linalg.norm(velDes[:3])*self.v_max

        ## If there are no nearby objects to avoid
        else: # Go to goal 
            velDes[:3] = self.goToGoalField()

        # Normalize velocity
        if np.linalg.norm(velDes[:3]) > self.v_max:
            velDes[:3] = velDes[:3]/np.linalg.norm(velDes[:3])*self.v_max
        
        # Heading Control
        w_d = self.headingControl(velDes)
        velDes[3] = w_d

        return velDes


    ## TODO: Implement in 3D
    ## For Now: 2D Implementation
    def getCloseObjects(self):
        closeObjects = []
        move = False

        # Perform transformed coordinates
        T_vo = self.transformToGoalCoords()    
        
        for obst in self.detections:
            obs_pos = np.array([obst.position.x, obst.position.y, obst.position.z])
            
            pos = np.array([obs_pos[0], obs_pos[1], 1])
            obst_trans = np.matmul(T_vo, pos)
            if obst_trans[1] > 0 and obst.distance < self.safe_dist:
                closeObjects.append(obst)
                move = True
        return closeObjects, move


    def changeGoalPt(self):
        dist_to_goal = np.linalg.norm(self.pos-self.goal)

        if(dist_to_goal < self.switch_dist):
            self.goalPt += 1
            if(self.goalPt > len(self.waypoints)-1):
                self.goalPt = 0
            self.goal =self.waypoints[self.goalPt]


    def headingControl(self, velDes):
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

        # Check if object is stationary
        if np.linalg.norm(trans_vel) > 50:
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



    def move(self):
        # Check if we have reached the next waypoint. If so, update
        rospy.loginfo("HERE")

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


    ## TODO: Implement in 3D
    ## For Now: 2D Implementation
    def transformToGoalCoords(self):

        dp = self.goal - self.pos

        th = np.arctan2(dp[1],dp[0]) - np.pi/2
        th = np.arctan2(np.sin(th), np.cos(th))
        R_ov = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
        t_ov = self.pos[:2]

        tempVect = np.matmul((-1 * R_ov.T),  t_ov)
        T_vo = np.array([[R_ov[0,0], R_ov[1,0], tempVect[0]], [R_ov[0,1], R_ov[1,1], tempVect[1]], [0, 0, 1]])

        return T_vo


    def updateDetections(self):
        in_detections = self.query_detections_service_(vehicle_position=self.pos_pt, attitude=self.quat)
        self.detections = in_detections.detection_array.detections


if __name__ == '__main__': 
  try:
    rospy.init_node('vectFieldController')

    start_pos = np.array([0,0,rospy.get_param('transform_z_offset')])

    # Launch Node
    field = vectFieldController()
    field.waypoints  = np.array([rospy.get_param('waypoint_1'), 
                                 rospy.get_param('waypoint_2')])
    field.goal = field.waypoints[0] 

    rospy.sleep(2)

    rospy.loginfo("LAUNCH")

    ########### Takeoff Controll ###############
    resp1 = field.takeoff_service(4)
    ########### Takeoff Controll ###############

    rospy.sleep(10)

    startTime = rospy.Time.now()
    rate = rospy.Rate(10) # 10hz
    while (rospy.Time.now() - startTime).to_sec() < 600:
        field.move()
        rate.sleep()

    rospy.loginfo("LAND")

    ########### Takeoff Controll ###############
    resp1 = field.takeoff_service(6)
    ########### Takeoff Controll ###############

    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass

