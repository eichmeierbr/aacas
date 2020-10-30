#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32, UInt8
from sensor_msgs.msg import Joy, NavSatFix, BatteryState, Imu
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped, PointStamped, Point, Vector3, Quaternion
from dji_m600_sim.srv import SimDroneTaskControl
from dji_sdk.srv import DroneTaskControl, SDKControlAuthority, SetLocalPosRef
from aacas_detection.srv import QueryDetections
from traj_prediction.msg import tracked_obj, tracked_obj_arr



class Objects:
    def __init__(self, pos = np.zeros(3), vel=np.zeros(3), dist = np.inf, id=0):
        self.id = id
        self.position = pos
        self.velocity = vel
        self.distance = dist
        self.last_orbit_change_ = rospy.Time.now() - rospy.Duration(secs=1000)
        self.orbit = -1

  

class vectFieldController:

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
        self.is_safe = True

        # safety check constraints
        self.x_constraint = rospy.get_param('x_constraint')
        self.y_constraint = rospy.get_param('y_constraint')
        self.is_ready = False # Before reaching 1st waypoint

        self.rc_axes = []
        self.gps_health = 0
        self.gps_position = NavSatFix()
        self.flight_status = 0
        self.battery_state = BatteryState()
        self.height_above_takeoff = 0
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
        rospy.Subscriber('/height_above_takeoff', Float32, self.height_above_takeoff_cb)
        rospy.Subscriber('/imu', Imu, self.imu_cb)


        tracked_obj_topic = rospy.get_param('obstacle_trajectory_topic')
        rospy.Subscriber(tracked_obj_topic, tracked_obj_arr, self.updateDetections, queue_size=1)

        # Service Information
        self.live_flight = rospy.get_param('live_flight',default=False)
        if self.live_flight:  
            takeoff_service_name = rospy.get_param('takeoff_land_service_name')
            rospy.wait_for_service(takeoff_service_name)
            self.takeoff_service = rospy.ServiceProxy(takeoff_service_name, DroneTaskControl)
    
            authority_service_name = 'dji_sdk/sdk_control_authority'
            rospy.wait_for_service(authority_service_name)
            self.authority_service = rospy.ServiceProxy(authority_service_name, SDKControlAuthority)

            set_pos_name = 'dji_sdk/set_local_pos_ref'
            rospy.wait_for_service(set_pos_name)
            self.set_pos_service = rospy.ServiceProxy(set_pos_name, SetLocalPosRef)


            rospy.Subscriber('/dji_sdk/rc', Joy, self.rc_cb)
            rospy.Subscriber('/dji_sdk/gps_health', UInt8, self.gps_health_cb)
            rospy.Subscriber('/dji_sdk/gps_position', NavSatFix, self.gps_position_cb)
            rospy.Subscriber('/dji_sdk/flight_status', UInt8, self.flight_status_cb)
            rospy.Subscriber('/dji_sdk/battery_state', BatteryState, self.battery_state_cb)
            #rospy.Subscriber('/dji_sdk/height_above_takeoff', Float32, self.height_above_takeoff_cb)
            rospy.Subscriber('/dji_sdk/angular_velocity_fused', Vector3Stamped, self.angular_vel_cb)
            rospy.Subscriber('/dji_sdk/acceleration_ground_fused', Vector3Stamped, self.acceleration_cb)

        else:
            takeoff_service_name = rospy.get_param('takeoff_land_service_name')
            rospy.wait_for_service(takeoff_service_name)
            self.takeoff_service = rospy.ServiceProxy(takeoff_service_name, SimDroneTaskControl)



    def rc_cb(self, msg):
        self.rc_axes = msg.axes

    def gps_health_cb(self, msg):
        self.gps_health = msg.data

    def gps_position_cb(self, msg):
        self.gps_position = msg
    
    def flight_status_cb(self, msg):
        self.flight_status = msg.data

    def battery_state_cb(self, msg):
        self.battery_state = msg

    def height_above_takeoff_cb(self, msg):
        self.height_above_takeoff = msg.data

    def angular_vel_cb(self, msg):
        vec = msg.vector
        self.angular_vel = np.array([vec.x, vec.y, vec.z])

    def acceleration_cb(self, msg):
        vec = msg.vector
        self.acceleration = np.array([vec.x, vec.y, vec.z])   

    def imu_cb(self, msg):
        lin = msg.linear_acceleration
        ang = msg.angular_velocity
        self.acceleration = np.array([lin.x, lin.y, lin.z])
        self.angular_vel = np.array([ang.x, ang.y, ang.z])

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


    ## TODO: Move find close obstacles to move. Do position control if no obstacles to avoid
    ## For now: Always do velocity control
    def move(self):

        # Check if we have reached the next waypoint. If so, update
        self.changeGoalPt()
        self.v_max =  rospy.get_param('maximum_velocity')
        
 
        # Get velocity vector
        velDes = self.getXdes() 

        # Pause to rotate after waypoint
        if (rospy.Time.now() - self.last_waypoint_time).to_sec() < self.waypoint_wait_time:
            velDes[:3] = [0,0,0]

        # Publish Vector
        joy_out = Joy()
        joy_out.header.stamp = rospy.Time.now()
        joy_out.axes = [velDes[0], velDes[1], velDes[2],velDes[3]]
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


    def safetyCheck(self):
        # Check if position and velocity violates safety constraints.
        # Position constraints are set in vector_field_planner_params.yaml
        # x_constraint: [xmin, xmax]
        # y_constraint: [ymin, ymax]
        # Velocity constraint is self.v_max (norm of all directions).

        position = self.pos

        if self.is_ready: # finished taking off, aiming for waypoints, then we take z velocity into account.
            velocity = np.sum(self.vel ** 2)
        else: # if still in takeoff progress, ignore z velocity.
            velocity = np.sum(self.vel[:2] ** 2)

        ## Assume the field is not safe
        field.is_safe = False

        #print(np.sum(velocity ** 2))
        ## Write the if statements to enter if something bad happens
        if not self.x_constraint[0] <= position[0] <= self.x_constraint[1]:
            rospy.logerr("Unsafe X Position: X=%.2f", position[0])
            
        elif not self.y_constraint[0] <= position[1] <= self.y_constraint[1]:
            rospy.logerr("Unsafe Y Position: Y=%.2f", position[1])

        elif not velocity <= 5:
            rospy.logerr("Unsafe Velocity: V=%.2f", velocity)

        elif abs(self.angular_vel[0]) > 0.01 or abs(self.angular_vel[1])  > 0.01 or abs(self.angular_vel[2]) > 3.0:
            rospy.logerr("Angular velocity too large, current angular velocities: %.2f, %.2f, %.2f", \
                self.angular_vel[0], self.angular_vel[1], self.angular_vel[2])

        elif abs(self.acceleration[0]) > 3.0 or abs(self.acceleration[1])  > 3.0 or abs(self.angular_vel[2]) > 10.0:
            rospy.logerr("Acceleration too large, current accelerations: %.2f, %.2f, %.2f", \
                self.acceleration[0], self.acceleration[1], self.acceleration[2])

        #elif self.height_above_takeoff > self.h_max:
        #    rospy.logerr("Drone flying too high, current height: %.2f", self.height_above_takeoff)

        #elif not self.rc_axes:
        #    rospy.logerr("Could not read remote control axes")

        #elif self.gps_health <= 3:
        #    rospy.logerr("Not Enough Satellites, current satellites: %d", self.gps_health)

        #elif self.battery_state.voltage < 21.0:
        #    rospy.logerr("Battery voltage too low, current voltage: %.2fV", self.battery_state.voltage)

        #elif self.battery_state.power_supply_health != 1:
            """
            UNKNOWN=0
            GOOD=1
            OVERHEAT=2
            DEAD=3
            OVERVOLTAGE=4
            UNSPEC_FAILURE=5
            COLD=6
            WATCHDOG_TIMER_EXPIRE=7
            SAFETY_TIMER_EXPIRE=8
            """
        #    rospy.logerr("Power supply health condition: %d", self.battery_state.power_supply_health)

        # elif self.gps_position check:
        #     pass

        else:
            field.is_safe = True

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
    rospy.init_node('vectFieldController')
    rate = rospy.Rate(10) # 10hz

    # Launch Node
    field = vectFieldController()

    x_waypoint = rospy.get_param('waypoint_x')
    y_waypoint = rospy.get_param('waypoint_y')
    z_waypoint = rospy.get_param('waypoint_z')
    field.waypoints = np.transpose(np.array([x_waypoint, y_waypoint, z_waypoint]))
    field.goal = field.waypoints[0] 

    rospy.sleep(2)


    ########### Takeoff Controll ###############
    if field.live_flight:

        ########### Wait for GPS Lock  #############
        ## TODO: Add functionality to check number of satellites
        rospy.loginfo("Getting Satellite Fix")
        while field.gps_health < 3:
            rate.sleep()
        rospy.loginfo("Obtained Satellite Fix")
        resp = field.authority_service(1)
        resp = field.set_pos_service()


    ########### Takeoff Controll ###############
    rospy.loginfo("LAUNCH")
    resp1 = field.takeoff_service(4)

    rospy.sleep(5)

    startTime = rospy.Time.now()
    while (rospy.Time.now() - startTime).to_sec() < 200 and field.is_safe:
        # if ((rospy.Time.now() - startTime).to_sec() >50): 
            # field.rush()
        # else: field.move()
        field.move()

        field.safetyCheck()
        #field.is_safe = True
        rate.sleep()
    
    if field.is_safe: # If the planner exited normally, land
        rospy.loginfo("LAND")
    
        ########### Landing Controll ###############
        resp1 = field.takeoff_service(6)
        ########### Landing Controll ###############

    else: # The planner detected unsafe conditions
        rospy.logerr("Unsafe condition detected. Hover for ten seconds.")
        ##
        ## Error protocol goes here
        field.hoverInPlace()
        rospy.sleep(10)
        rospy.logerr("Unsafe condition detected. Land and relaunch to restart.")
        resp1 = field.takeoff_service(6)
        ##

    if field.live_flight:
        resp1 = field.authority_service(0)


    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass

