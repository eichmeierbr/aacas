#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from scipy import interpolate
from nav_msgs.msg import Path
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped, PointStamped, Point, Vector3, Quaternion
from lidar_process.msg import tracked_obj, tracked_obj_arr


def moving_average(a, n=3) :
    ret = np.cumsum(a, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    return ret[n - 1:] / n


class Objects:
    def __init__(self, pos = np.zeros(3), vel=np.zeros(3), dist = np.inf, id=0):
        self.id = id
        self.position = pos
        self.velocity = vel
        self.distance = dist
        self.last_orbit_change_ = rospy.Time.now() - rospy.Duration(secs=1000)
        self.orbit = -1

  

class pathGenerator:

    def __init__(self, waypoints = [[0,0,0]]):
        self.v_max =  rospy.get_param('maximum_velocity')
        self.detections = []

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

        # Go to Goal Parameters
        self.g2g_sig =  rospy.get_param('g2g_sigma')
        self.g2g_sig_sq = self.g2g_sig**2

        # state Information
        self.pos = np.zeros(3)
        self.now_pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.yaw = 0
        self.quat = Quaternion()
        self.pos_pt = Point()

        # Publisher Information
        self.future_path_pub_ = rospy.Publisher('future_path', Path, queue_size=10)


        # Subscriber Information
        rospy.Subscriber(rospy.get_param('position_pub_name'), PointStamped,      self.position_callback, queue_size=1)
        rospy.Subscriber(rospy.get_param('velocity_pub_name'), Vector3Stamped,    self.velocity_callback, queue_size=1)
        rospy.Subscriber(rospy.get_param('attitude_pub_name'), QuaternionStamped, self.attitude_callback, queue_size=1)

        tracked_obj_topic = rospy.get_param('obstacle_trajectory_topic')
        rospy.Subscriber(tracked_obj_topic, tracked_obj_arr, self.updateDetections, queue_size=1)




    def position_callback(self, msg):
        pt = msg.point
        self.pos_pt = pt
        self.now_pos = np.array([pt.x, pt.y, pt.z])

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


    ## TODO: Implement in 3D
    ## For Now: 2D Implementation
    def getXdes(self):
        velDes = np.zeros(4)

        # Check if we are close to an object
        closeObjects, avoid = self.getCloseObjects()
        
        # If close to object, orbit
        if avoid:
            vels = []
            for obstacle in closeObjects:
                if self.change_orbit_wait_ < (rospy.Time.now() - obstacle.last_orbit_change_).to_sec():
                    self.decideOrbitDirection(obstacle)
                else:
                    self.freq = obstacle.orbit
                # velDes[:3] = self.getOrbit([obstacle.position.x,obstacle.position.y,obstacle.position.z])

                # self.decideOrbitDirection(obstacle)
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



    ## TODO: Need to handle moving obstacles better
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
        ob.last_orbit_change_ = self.last_orbit_change_
        ob.orbit = self.freq



    def getPath(self):

        self.v_max =  rospy.get_param('maximum_velocity')
        self.pos = np.array(self.now_pos)


        ## TODO Perform integration
        path = np.array(self.pos)

        for i in range(150):
            self.simDetections()
            velDes = self.getXdes() 
            self.pos += velDes[:3]*0.1
            path = np.vstack((path,self.pos))

        self.plotPath(path)
        self.now_pos = path[1]





    def plotPath(self, path):
        xs = path[:,0]
        ys = path[:,1]

        xs = moving_average(xs, n=25)
        ys = moving_average(ys, n=25)

        plt.clf()
        plt.plot(xs, ys)
        for i in range(len(self.detections)):
            plt.scatter(self.detections[i].position.x, self.detections[i].position.y, marker='o', c='r')
        plt.scatter(self.now_pos[0], self.now_pos[1], marker='s', c='b', s=np.power(30,2))
        plt.scatter(self.goal[0], self.goal[1], marker='^', c='g', s=80)
        plt.xlim([-10,10])
        plt.ylim([-5,25])

        plt.draw()
        plt.pause(0.0001)

        a=4



        




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
        self.detections = []
        for obj in in_detections:
            newObj = Objects()
            newObj.position = obj.point
            newObj.velocity = Point(0,0,0)
            newObj.id = obj.object_id
            newObj.distance = np.linalg.norm([obj.point.x - self.pos[0], obj.point.y - self.pos[1], obj.point.z - self.pos[2]])
            self.detections.append(newObj)



    def simDetections(self):
        self.detections = []
        in_detections = [[0,10,2]]
        for obj in in_detections:
            newObj = Objects()
            newObj.position = Point(obj[0], obj[1], obj[2])
            newObj.velocity = Point(0,0,0)
            newObj.distance = np.linalg.norm([newObj.position.x - self.pos[0], newObj.position.y - self.pos[1], newObj.position.z - self.pos[2]])
            self.detections.append(newObj)







if __name__ == '__main__': 
  try:
    rospy.init_node('path_generation')
    rate = rospy.Rate(10) # 10hz

    # Launch Node
    path_gen = pathGenerator()
    path_gen.simDetections()

    x_waypoint = rospy.get_param('waypoint_x')
    y_waypoint = rospy.get_param('waypoint_y')
    z_waypoint = rospy.get_param('waypoint_z')
    path_gen.waypoints = np.transpose(np.array([x_waypoint, y_waypoint, z_waypoint]))
    path_gen.goal = path_gen.waypoints[1] 


    while not rospy.is_shutdown():
        path_gen.getPath()
        rate.sleep()
    
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass