#!/usr/bin/env python

import rospy
import copy
import itertools
import numpy as np
from sensor_msgs.msg import Joy
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped, PointStamped, Point, Vector3, Quaternion, PoseStamped
from nav_msgs.msg import Path
from traj_prediction.msg import tracked_obj, tracked_obj_arr


def moving_average(a, n=3) :
    ret = np.cumsum(a, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    return ret[n - 1:] / n


class ObjMulti:
    def __init__(self, pos = np.zeros(3), vel=np.zeros(3), acc=np.zeros(3), dist = np.inf, id=0):
        self.id = id
        self.position = pos
        self.velocity = vel
        self.acceleration = acc
        self.distance = dist
        self.last_orbit_change_ = rospy.Time.now() - rospy.Duration(secs=1000)
        self.orbit = 1
  

class vectFieldController:

    def __init__(self, waypoints = [[0,0,0]]):
        self.v_max =  rospy.get_param('maximum_velocity')
        self.detections = []
        self.hold_detections = []
        self.in_detections = []
        self.lastPathPub = rospy.Time.now()
        self.lastPathCalc= rospy.Time.now()
        self.path = []
        self.curr_path = []

        # Waypoint params
        self.waypoints = waypoints
        self.goalPt = 0
        self.goal = self.waypoints[self.goalPt]
        self.switch_dist =  rospy.get_param('switch_waypoint_distance')
        self.last_waypoint_time = rospy.Time.now()
        self.waypoint_wait_time = rospy.get_param('waypoint_wait', default=5.0)

        # Orbit params
        self.freq = -1 # Orbit direction (+: CW, -: ccw)
        self.safe_dist = rospy.get_param('safe_distance')
        self.rad = self.safe_dist # Radius of orbit
        self.k_conv =  rospy.get_param('orbit_k_conv') # Gain to converge to orbit
        self.K_theta =  rospy.get_param('heading_k_theta')
        self.last_orbit_change_ = rospy.Time.now()
        self.change_orbit_wait_ = rospy.get_param('orbit_change_wait')
        self.orbits = {}

        # Go to Goal Parameters
        self.g2g_sig =  rospy.get_param('g2g_sigma')
        self.g2g_sig_sq = self.g2g_sig**2

        # state Information
        self.pos = np.zeros(3)
        self.curr_pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.curr_vel = np.zeros(3)
        self.yaw = 0
        self.curr_yaw = 0
        self.yaw_rate = 0
        self.quat = Quaternion()
        self.pos_pt = Point()

        self.is_ready = False # Before reaching 1st waypoint

        self.height_above_takeoff = 0
        self.angular_vel = np.zeros(3)
        self.acceleration = np.zeros(3)


        # Publisher Information
        self.vel_ctrl_pub_ = rospy.Publisher('aacas_velocity', Joy, queue_size=10)

        self.goal_pub_ = rospy.Publisher('current_goal', Point, queue_size=10)
        self.future_path_pub_ = rospy.Publisher('future_path', Path, queue_size=10)
        

        # Subscriber Information
        rospy.Subscriber(rospy.get_param('position_pub_name'), PointStamped,      self.position_callback, queue_size=1)
        rospy.Subscriber(rospy.get_param('velocity_pub_name'), Vector3Stamped,    self.velocity_callback, queue_size=1)
        rospy.Subscriber(rospy.get_param('attitude_pub_name'), QuaternionStamped, self.attitude_callback, queue_size=1)

        tracked_obj_topic = rospy.get_param('obstacle_trajectory_topic')
        rospy.Subscriber(tracked_obj_topic, tracked_obj_arr, self.updateDetections, queue_size=1)


        # rospy.Subscriber('/dji_sdk/height_above_takeoff', Float32, self.height_above_takeoff_cb)
        # rospy.Subscriber('/dji_sdk/angular_velocity_fused', Vector3Stamped, self.angular_vel_cb)
        # rospy.Subscriber('/dji_sdk/acceleration_ground_fused', Vector3Stamped, self.acceleration_cb)



    def angular_vel_cb(self, msg):
        vec = msg.vector
        self.angular_vel = np.array([vec.x, vec.y, vec.z])

    def acceleration_cb(self, msg):
        vec = msg.vector
        self.acceleration = np.array([vec.x, vec.y, vec.z])   

    def position_callback(self, msg):
        pt = msg.point
        self.pos_pt = pt
        self.curr_pos = np.array([pt.x, pt.y, pt.z])

    def velocity_callback(self, msg):
        pt = msg.vector
        self.curr_vel = np.array([pt.x, pt.y, pt.z])

    def attitude_callback(self, msg):
        q = msg.quaternion
        self.quat = q

        qt = [q.w, q.x, q.y, q.z]
        [yaw, pitch, roll] = self.yaw_pitch_roll(qt)
        self.curr_yaw = yaw


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


    ## TODO: Implement in 3D
    ## For Now: 2D Implementation
    def getXdes(self, switchGoal=True):
        velDes = np.zeros(4)

        if len(self.waypoints) == 0: return velDes
        
        # Check if we are close to an object
        closeObjects, avoid = self.getCloseObjects()
        
        # If close to object, orbit
        if avoid:
            vels = []
            for obstacle in closeObjects:
                self.freq = self.orbits[obstacle.id]
                vel = self.getOrbit(obstacle.position)
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
            # obs_pos = np.array([obst.position.x, obst.position.y, obst.position.z])
            
            pos = np.array([obst.position[0], obst.position[1], 1])
            obst_trans = np.matmul(T_vo, pos)
            if obst_trans[1] > -0.5 and obst.distance < self.safe_dist:
                closeObjects.append(obst)
                move = True
        return closeObjects, move


    def changeGoalPt(self):
        dist_to_goal = np.linalg.norm(self.pos-self.goal)

        if(dist_to_goal < self.switch_dist):
            self.is_ready = True #set true after reaching 1st waypoint and allow z velocity safety checks.
            self.goalPt += 1
            if(self.goalPt > len(self.waypoints)-1):
                self.goalPt = 0
            self.goal =self.waypoints[self.goalPt]
            self.last_waypoint_time = rospy.Time.now()
            self.goal_pub_.publish(Point(self.goal[0], self.goal[1], self.goal[2]))


    def headingControl(self, velDes):
        vel_angle = np.arctan2(velDes[1], velDes[0])
        angleDiff = vel_angle - self.yaw
        angleDiff = (angleDiff + np.pi) % (2 * np.pi) - np.pi
        w_d = self.K_theta * angleDiff

        return w_d


    def getVelDes(self):
        velDes = np.zeros(4)

        if len(self.hold_detections) == 0:
            velDes[:3] = self.goToGoalField()

            # Heading Control
            w_d = self.headingControl(velDes)
            velDes[3] = w_d
        else:
            velDes = self.getXdes()

        # Normalize velocity
        if np.linalg.norm(velDes[:3]) > self.v_max:
            velDes[:3] = velDes[:3]/np.linalg.norm(velDes[:3])*self.v_max
        

        return velDes


    ## TODO: Move find close obstacles to move. Do position control if no obstacles to avoid
    ## For now: Always do velocity control
    def move(self):
        # Check if we have reached the next waypoint. If so, update
        self.changeGoalPt()
        self.v_max =  rospy.get_param('maximum_velocity')

        self.hold_detections = copy.copy(self.in_detections)
        self.resetState()

        # Only update the path every second
        if (rospy.Time.now() - self.lastPathCalc).to_sec() > 0.5:
            bestCost = np.inf
            bestPath = []
            bestOrbits = copy.copy(self.orbits)
            orbitPairs = map(list, itertools.product([-1, 1], repeat=len(self.detections)))
            # Iterate through all orbit pairs
            if len(orbitPairs) > 1:
                for orbits in orbitPairs:
                    self.setOrbits(orbits)
                    cost, path = self.findPath()
    
                    # Save the best path
                    if cost < bestCost:
                        bestCost = cost
                        bestPath = path
                        bestOrbits = copy.copy(self.orbits)
        
                    self.resetState()
        
                self.publishPath(bestPath)
                self.resetState()
                self.lastPathPub = rospy.Time.now()
                self.lastPathCalc= rospy.Time.now()
                self.curr_path = bestPath
                self.orbits = copy.copy(bestOrbits)
        

        # if updateOrbits:
        # if len(self.orbits)>0:
        #     # print(self.orbits)
        #     self.detections = self.hold_detections
        #     self.setOrbits(self.orbits, isUpdate=True)
    
        ## TODO: Need to map orbits to obstacle ids. Otherwise this wipes to a CCW orbit
        # velDes = self.getXdes()
        velDes = self.getVelDes()


        # Pause to rotate after waypoint
        if (rospy.Time.now() - self.last_waypoint_time).to_sec() < self.waypoint_wait_time:
            velDes[:3] = [0,0,0]

        # Publish Vector
        joy_out = Joy()
        joy_out.header.stamp = rospy.Time.now()
        joy_out.axes = [velDes[0], velDes[1], velDes[2],velDes[3]]
        self.vel_ctrl_pub_.publish(joy_out)


    def resetState(self):
        self.yaw = copy.copy(self.curr_yaw)       
        self.vel = copy.copy(self.curr_vel)       
        self.pos = copy.copy(self.curr_pos)     
        self.detections = copy.copy(self.hold_detections)
 

    def setOrbits(self, orbitPair, isUpdate=False):
        # if isUpdate:
        #     for i in range(len(self.in_detections)):
        #         self.in_detections[i].orbit = self.orbits[self.in_detections[i].id]
        # else:
        # orbit_dict = {}
        for i in range(len(self.detections)):
            self.detections[i].orbit = orbitPair[i]
            self.orbits[self.detections[i].id] = orbitPair[i]
        # self.orbits = orbit_dict


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


    def updateDetections(self, msg):
        in_detections = msg.tracked_obj_arr
        self.in_detections = []
        for obj in in_detections:
            newObj = ObjMulti()
            newObj.position = np.array([obj.point.x, obj.point.y, obj.point.z])
            newObj.velocity = np.array([obj.vel.x, obj.vel.y, obj.vel.z])
            newObj.acceleration= np.array([0,0,0])
            newObj.id = obj.object_id
            newObj.distance = np.linalg.norm([newObj.position - self.pos])
            self.in_detections.append(newObj)


    def simDetections(self, dt):
        for i in range(len(self.detections)):
            self.detections[i].position = self.detections[i].position + self.detections[i].velocity*dt
            self.detections[i].velocity = self.detections[i].velocity + self.detections[i].acceleration*dt
            self.detections[i].distance = np.linalg.norm([self.detections[i].position - self.pos])
            self.closestObjDist = min(self.detections[i].distance, self.closestObjDist)


    
    def findPath(self):
        dt = 0.25
        timeHorizon = 8
        self.path = []
        self.closestObjDist = np.inf
        self.detections = copy.copy(self.in_detections)

        # Perform path simulation
        for t in np.arange(0,timeHorizon, dt):
            # Append State
            self.path.append(self.pos)

            velDes = self.getXdes()
            velError = velDes[:3] - self.vel
            self.vel += velError * dt
            self.yaw = np.arctan2(velDes[1], velDes[0])
            self.pos = self.pos + self.vel[:3]*dt
            self.simDetections(dt)

        # Smooth path
        self.path = np.array(self.path)
        xs = moving_average(self.path[:,0], n=5)
        ys = moving_average(self.path[:,1], n=5)
        zs = moving_average(self.path[:,2], n=5)
        self.path = np.vstack((xs,ys,zs)).transpose()
        self.path = np.vstack((self.curr_pos, self.path))


        cost = self.calcPathCost()
        return cost, copy.copy(self.path)



    def calcPathCost(self):
        toGoal = np.linalg.norm(self.path[-1] - self.goal)
        # return toGoal
        return np.abs(1/(self.closestObjDist-0.5)) + 3*toGoal


    def publishPath(self, in_path):
        path = []
        for pos in in_path:
            newPose = PoseStamped()
            newPose.header.frame_id = 'local'
            newPose.pose.position = Point(pos[0], pos[1], pos[2])
            path.append(newPose)

        # newPose = PoseStamped()
        # newPose.header.frame_id = 'local'
        # newPose.pose.position = Point(self.pos[0], self.pos[1], self.pos[2])
        # path.append(newPose)

        out = Path()
        out.poses = path
        out.header.frame_id = 'local'
        out.header.stamp = rospy.Time.now()
        self.future_path_pub_.publish(out)


if __name__ == '__main__': 
  try:
    rospy.init_node('vectFieldMulti')
    rate = rospy.Rate(10) # 10hz

    # Launch Node
    field = vectFieldController()

    x_waypoint = rospy.get_param('waypoint_x')
    y_waypoint = rospy.get_param('waypoint_y')
    z_waypoint = rospy.get_param('waypoint_z')
    field.waypoints = np.transpose(np.array([x_waypoint, y_waypoint, z_waypoint]))
    field.goal = field.waypoints[0] 

    while not rospy.is_shutdown():
        field.move()
        rate.sleep()
    
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass

