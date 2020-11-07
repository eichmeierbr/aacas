#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32, UInt8
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
        self.vel = np.zeros(3)
        self.yaw = 0
        self.pitch=0
        self.roll =0
        self.yaw_rate = 0
        self.quat = Quaternion()
        self.pos_pt = Point()
        self.is_safe = False

        # safety check constraints
        self.x_constraint   = rospy.get_param('x_constraint', default=[-15, 15])
        self.y_constraint   = rospy.get_param('y_constraint', default=[-5, 5])
        self.z_constraint   = rospy.get_param('z_constraint', default=[0.5, 15])
        self.v_max          = rospy.get_param('unsafe_velocity', default=5)
        self.max_tilt       = rospy.get_param('max_pitch_roll', default=1.05)
        self.min_gps        = rospy.get_param('min_gps_health', default=4)
        self.is_ready = False # Before reaching 1st waypoint

        ## DJI Topic Variables 
        self.rc_axes = []
        self.gps_health = 0
        self.gps_position = NavSatFix()
        self.flight_status = 0
        self.battery_state = BatteryState()
        self.height_above_takeoff = 0
        self.angular_vel = np.zeros(3)
        self.acceleration = np.zeros(3)

        ## Sensor Delay Times
        self.pos_delay          = rospy.get_param('position_delay')
        self.vel_delay          = rospy.get_param('velocity_delay')
        self.gps_delay          = rospy.get_param('gps_delay')
        self.attitude_delay     = rospy.get_param('attitude_delay')
        self.battery_delay      = rospy.get_param('battery_delay')
        self.rc_delay           = rospy.get_param('rc_delay')

        ## DJI Topic Times
        self.last_rc            = rospy.Time.now()
        self.last_gps_health    = rospy.Time.now()
        self.last_gps_pos       = rospy.Time.now()
        self.last_battery       = rospy.Time.now()
        self.last_height        = rospy.Time.now()
        self.last_pos           = rospy.Time.now()
        self.last_vel           = rospy.Time.now()
        self.last_angular_vel   = rospy.Time.now()
        self.last_acceleration  = rospy.Time.now()
        self.last_attitude      = rospy.Time.now()


        # Publisher Information
        vel_ctrl_pub_name = rospy.get_param('vel_ctrl_sub_name')
        self.vel_ctrl_pub_ = rospy.Publisher(vel_ctrl_pub_name, Joy, queue_size=10)

        pos_ctrl_pub_name = rospy.get_param('pos_ctrl_sub_name')
        self.pos_ctrl_pub_ = rospy.Publisher(pos_ctrl_pub_name, Joy, queue_size=10)

        # self.goal_pub_ = rospy.Publisher('current_goal', Point, queue_size=10)
        

        # Subscriber Information
        rospy.Subscriber('aacas_velocity', Joy, self.command_vel_callback, queue_size=1)

        tracked_obj_topic = rospy.get_param('obstacle_trajectory_topic')
        rospy.Subscriber(tracked_obj_topic, tracked_obj_arr, self.updateDetections, queue_size=1)

        rospy.Subscriber(rospy.get_param('position_pub_name'), PointStamped,      self.position_callback, queue_size=1)
        rospy.Subscriber(rospy.get_param('velocity_pub_name'), Vector3Stamped,    self.velocity_callback, queue_size=1)
        rospy.Subscriber(rospy.get_param('attitude_pub_name'), QuaternionStamped, self.attitude_callback, queue_size=1)
        rospy.Subscriber('/height_above_takeoff', Float32, self.height_above_takeoff_cb)
        # rospy.Subscriber('/imu', Imu, self.imu_cb)
        rospy.Subscriber('/dji_sdk/rc', Joy, self.rc_cb)
        rospy.Subscriber('/dji_sdk/gps_health', UInt8, self.gps_health_cb)
        rospy.Subscriber('/dji_sdk/gps_position', NavSatFix, self.gps_position_cb)
        rospy.Subscriber('/dji_sdk/flight_status', UInt8, self.flight_status_cb)
        rospy.Subscriber('/dji_sdk/battery_state', BatteryState, self.battery_state_cb)
        #rospy.Subscriber('/dji_sdk/height_above_takeoff', Float32, self.height_above_takeoff_cb)
        rospy.Subscriber('/dji_sdk/angular_velocity_fused', Vector3Stamped, self.angular_vel_cb)
        rospy.Subscriber('/dji_sdk/acceleration_ground_fused', Vector3Stamped, self.acceleration_cb)



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



    def rc_cb(self, msg):
        self.rc_axes = msg.axes
        self.last_rc = rospy.Time.now()

    def gps_health_cb(self, msg):
        self.gps_health = msg.data
        self.last_gps_health = rospy.Time.now()

    def gps_position_cb(self, msg):
        self.gps_position = msg
        self.last_gps_pos = rospy.Time.now()
    
    def flight_status_cb(self, msg):
        self.flight_status = msg.data

    def battery_state_cb(self, msg):
        self.battery_state = msg
        self.last_battery = rospy.Time.now()

    def height_above_takeoff_cb(self, msg):
        self.height_above_takeoff = msg.data
        self.last_height = rospy.Time.now()

    def angular_vel_cb(self, msg):
        vec = msg.vector
        self.angular_vel = np.array([vec.x, vec.y, vec.z])
        self.last_angular_vel = rospy.Time.now()

    def acceleration_cb(self, msg):
        vec = msg.vector
        self.acceleration = np.array([vec.x, vec.y, vec.z])
        self.last_acceleration = rospy.Time.now()

    def imu_cb(self, msg):
        lin = msg.linear_acceleration
        ang = msg.angular_velocity
        self.acceleration = np.array([lin.x, lin.y, lin.z])
        self.angular_vel = np.array([ang.x, ang.y, ang.z])

    def position_callback(self, msg):
        pt = msg.point
        self.pos_pt = pt
        self.pos = np.array([pt.x, pt.y, pt.z])
        self.last_pos = rospy.Time.now()

    def velocity_callback(self, msg):
        pt = msg.vector
        self.vel = np.array([pt.x, pt.y, pt.z])
        self.last_vel = rospy.Time.now()

    def attitude_callback(self, msg):
        q = msg.quaternion
        self.quat = q

        qt = [q.w, q.x, q.y, q.z]
        [yaw, pitch, roll] = self.yaw_pitch_roll(qt)
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll

        self.last_attitude = rospy.Time.now()


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
        if self.is_safe:
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





    def velocityCheck(self):
        # Velocity constraint is self.v_max (norm of all directions).
        if self.is_ready: # finished taking off, aiming for waypoints, then we take z velocity into account.
            velocity = np.linalg.norm(self.vel)
        else: # if still in takeoff progress, ignore z velocity.
            velocity = np.linalg.norm(self.vel[:2])

        if velocity > self.v_max:
            rospy.logerr("Unsafe Velocity: V=%.2f", velocity)
            return False

        if (rospy.Time.now() - self.last_vel).to_sec() > self.vel_delay:
            rospy.logwarn_throttle(1, 'Lost Velocity Signal')

        return True


    def xPositionCheck(self):
        # Position constraints are set in vector_field_planner_params.yaml
        # x_constraint: [xmin, xmax]
        if np.any([self.x_constraint[0] > self.pos[0], self.pos[0] > self.x_constraint[1]]):
            rospy.logerr("Unsafe X Position: X=%.2f", self.pos[0])
            return False

        if (rospy.Time.now() - self.last_pos).to_sec() > self.pos_delay:
            rospy.logerr(1, 'Lost Position Signal')
            return False

        return True
            

    def yPositionCheck(self):
        # Position constraints are set in vector_field_planner_params.yaml
        # y_constraint: [ymin, ymax]
        if np.any([self.y_constraint[0] > self.pos[1], self.pos[1] > self.y_constraint[1]]):
            rospy.logerr("Unsafe X Position: Y=%.2f", self.pos[1])
            return False

        return True


    def angularVelocityCheck(self):
        return True
        # elif abs(self.angular_vel[0]) > 0.01 or abs(self.angular_vel[1])  > 0.01 or abs(self.angular_vel[2]) > 3.0:
            # rospy.logerr("Angular velocity too large, current angular velocities: %.2f, %.2f, %.2f", \
                # self.angular_vel[0], self.angular_vel[1], self.angular_vel[2])


    def accelerationCheck(self):
        return True

        # elif abs(self.acceleration[0]) > 3.0 or abs(self.acceleration[1])  > 3.0 or abs(self.angular_vel[2]) > 10.0:
        #     rospy.logerr("Acceleration too large, current accelerations: %.2f, %.2f, %.2f", \
        #         self.acceleration[0], self.acceleration[1], self.acceleration[2])



    ## TODO: Verify height above takeoff works. Consider replacing with self.pos[2]
    def heightAboveTakeoffCheck(self):
        # if self.heightAboveTakeoffCheck > self.h_max:
        if self.pos[2] > self.z_constraint[1]:
            rospy.logerr("Drone too high. Current height: %.2f", self.height_above_takeoff)
            return False
        # elif self.height_above_takeoff < 0.5:
        elif self.pos[2] < self.z_constraint[0]:
            rospy.logerr("Drone too low. Current height: %.2f", self.height_above_takeoff)
            return False
        return True


    def attitudeCheck(self):
        if np.any([self.roll > self.max_tilt, self.pitch>self.max_tilt]):
            rospy.logerr('Unsafe Attitude. Tilt=%f' %(180/np.pi*max(self.roll, self.pitch)))
            return False

        if (rospy.Time.now() - self.last_attitude).to_sec() > self.attitude_delay:
            rospy.logerr('Lost Attitude Signal')
            return False

        return True

    def rcCheck(self):
        if (rospy.Time.now() - self.last_rc).to_sec() > self.rc_delay:
            rospy.logerr('Lost RC Signal')
            return False
        return True
        #elif not self.rc_axes:
        #    rospy.logerr("Could not read remote control axes")


    def gpsHealthCheck(self):
        if self.gps_health < self.min_gps:
           rospy.logerr(self,"Weak GPS, current signal: %d/5", self.gps_health)
           return False

        if (rospy.Time.now() - self.last_gps).to_sec() > self.gps_delay:
            rospy.logerr('Lost GPS Signal')
            return False
        return True


    def batteryStateCheck(self):
        if (rospy.Time.now() - self.last_battery).to_sec() > self.battery_delay:
            rospy.logwarn_throttle(5, 'Lost Battery Signal')
        return True
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


    def gpsPositionCheck(self):
        return True
        # elif self.gps_position check:
        #     pass





    def safetyCheck(self):
        # Check various DJI Topics for unsafe conditions
        # Mark field as unsafe if any check fails
        self.is_safe = np.all([ self.xPositionCheck(),
                                self.yPositionCheck(),
                                self.heightAboveTakeoffCheck(),
                                self.velocityCheck(),
                                self.attitudeCheck(),
                                # self.angularVelocityCheck(),
                                # self.accelerationCheck(),
                                # self.rcCheck(),
                                self.gpsHealthCheck(),
                                # self.gpsPositionCheck(),
                                self.batteryStateCheck()
        ])




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
    while pilot.gps_health < 3:
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
    

    ########### Takeoff Control ###############
    rospy.loginfo("LAUNCH")
    resp1 = pilot.takeoff_service(4)

    rospy.sleep(5)

    pilot.safetyCheck()

    startTime = rospy.Time.now()
    while (rospy.Time.now() - startTime).to_sec() < 200 and pilot.is_safe:
        pilot.safetyCheck()
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

