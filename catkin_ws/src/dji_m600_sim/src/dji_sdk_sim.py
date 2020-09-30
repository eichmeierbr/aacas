#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32, Header
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped, PointStamped, Point, Vector3, Quaternion, PoseStamped
from sensor_msgs.msg import Imu, Joy
from nav_msgs.msg import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
from dji_m600_sim.srv import SimDroneTaskControl, SimDroneTaskControlResponse

# from PACKAGE_NAME.srv import SERVICE1, SERVICE2, etc


class DJI_simulator:
  def __init__(self, pos_init=np.zeros(3), yaw_init=0):

    ## Last Update Time
    self.last_update_time_ = rospy.Time.now()

    ## State Parameters
    self.pos_ = pos_init
    self.vel_ = np.zeros(3)
    self.acc_ = np.zeros(3)
    self.yaw_ = yaw_init
    self.yaw_rate_ = 0

    # Initialize desired state values
    self.pos_des_ = self.pos_
    self.vel_des_ = self.vel_
    self.yaw_des_ = self.yaw_
    self.yaw_rate_des_ = self.yaw_rate_
    self.is_vel_ctrl_ = True

    ## Position Control Parameters
    A_x = rospy.get_param('pos_A_x')
    A_y = rospy.get_param('pos_A_y')
    A_z = rospy.get_param('pos_A_z')
    B_u = rospy.get_param('pos_B_u')
    B_v = rospy.get_param('pos_B_v')
    B_w = rospy.get_param('pos_B_w')
    K_pos = rospy.get_param('pos_K_pos_coeff')
    K_vel = rospy.get_param('pos_K_vel_coeff')
    self.v_max =  rospy.get_param('maximum_velocity')



    self.A_pos_ = np.zeros([6,6])
    self.A_pos_[:3,-3:] = np.diag([A_x, A_y, A_z])
    self.B_pos_ = np.zeros([6,3])
    self.B_pos_[-3:] = np.diag([B_u, B_v, B_w])

    self.K_pos_ = np.hstack((K_pos*np.identity(3), K_vel*np.identity(3)))
    self.K_yaw_ = rospy.get_param('K_yaw')


    ## Velocity Control Parameters
    B_u = rospy.get_param('vel_B_u')
    B_v = rospy.get_param('vel_B_v')
    B_w = rospy.get_param('vel_B_w')

    self.A_vel_ = np.zeros([3,3])
    self.B_vel_ = np.diag([1, 1, 1])
    self.K_vel_ = np.diag([1,1,1])
    self.K_yaw_rate_ = rospy.get_param('K_yaw_rate')


    ##  Publisher Information
    self.attitude_pub_name = rospy.get_param('attitude_pub_name')
    self.imu_pub_name = rospy.get_param('imu_pub_name')
    self.velocity_pub_name = rospy.get_param('velocity_pub_name')
    self.position_pub_name = rospy.get_param('position_pub_name')
    self.height_pub_name = rospy.get_param('height_pub_name')

    self.attitude_pub_ = rospy.Publisher(self.attitude_pub_name, QuaternionStamped, queue_size=10)
    self.imu_pub_ = rospy.Publisher(self.imu_pub_name, Imu, queue_size=10)
    self.velocity_pub_ = rospy.Publisher(self.velocity_pub_name, Vector3Stamped, queue_size=10)
    self.position_pub_ = rospy.Publisher(self.position_pub_name, PointStamped, queue_size=10)
    self.height_pub_ = rospy.Publisher(self.height_pub_name, Float32, queue_size=10)


    ## Subscriber Information
    self.pos_ctrl_sub_name = rospy.get_param('pos_ctrl_sub_name')
    self.vel_ctrl_sub_name = rospy.get_param('vel_ctrl_sub_name')

    # rospy.Subscriber(self.pos_ctrl_sub_name, Joy, self.pos_ctrl_callback, queue_size=10)
    rospy.Subscriber(self.vel_ctrl_sub_name, Joy, self.vel_ctrl_callback, queue_size=1)

     ## Service Server Information
    self.task_ctrl_service_ = rospy.Service(rospy.get_param('takeoff_land_service_name'), SimDroneTaskControl, self.DroneTaskControl)




# TODO Implement Position Control
  def pos_ctrl_callback(self, received_data):
    # Parse Input
    # self.last_update_time_ = received_data.Header.stamp
    self.pos_des_ = received_data.axes[:3]
    self.yaw_des_ = received_data.axes[3]
    self.is_vel_ctrl_ = False
    self.performMotion() ###### CONSIDER REMOVING THIS LINE #######




  def vel_ctrl_callback(self, received_data):
    # Parse Input
    # self.last_update_time_ = received_data.Header.stamp
    self.vel_des_ = received_data.axes[:3]
    self.yaw_rate_des_ = received_data.axes[3]
    self.is_vel_ctrl_ = True
    self.performMotion() ###### CONSIDER REMOVING THIS LINE #######
  

  def publishData(self):    
    head = Header()
    head.stamp = rospy.Time.now()
    head.frame_id = 'world'

    # Publish Attitude
    myQuat = self.getQuat()
    outQuat = QuaternionStamped()
    outQuat.header = head
    outQuat.quaternion = myQuat
    self.attitude_pub_.publish(outQuat)

    # Publish IMU
    imuOut = Imu()
    imuOut.header = head
    imuOut.orientation = myQuat
    imuOut.angular_velocity = Vector3(x=0.0, y=0.0, z=self.yaw_rate_)
    imuOut.linear_acceleration = Vector3(x=self.acc_[0], y=self.acc_[1], z=self.acc_[2] + 9.81)
    self.imu_pub_.publish(imuOut)

    # Publish Velocity
    velOut = Vector3Stamped()
    velOut.header = head
    velOut.vector = Vector3(x=self.vel_[0],y=self.vel_[1],z=self.vel_[2])
    self.velocity_pub_.publish(velOut)

    # Publish Position
    posStamp = PointStamped()
    posStamp.header = head
    myPoint = Point(x=self.pos_[0],y=self.pos_[1], z=self.pos_[2])
    posStamp.point = myPoint
    self.position_pub_.publish(posStamp)

    # Publish Height Above Ground
    height = Float32(self.pos_[-1])
    # height.data = self.pos_(-1)
    self.height_pub_.publish(height)



  # TODO: Add in smart way to play out the quaternion
  # FOR NOW: Always return orientation straight up
  def getQuat(self):
    quat = Quaternion()

    r = R.from_euler('xyz', [0, 0, self.yaw_])
    [x,y,z,w] = r.as_quat()

    quat.x = x
    quat.y = y
    quat.z = z
    quat.w = w
    return quat


  def performMotion(self):
    oldTime = self.last_update_time_
    self.last_update_time_ = rospy.Time.now()
    dt = oldTime - self.last_update_time_
    dt = abs(dt.to_sec())
    if dt > 1: return

    # If we are in Velocity Control
    if self.is_vel_ctrl_:
      # calculate errors
      vel_error = self.vel_ - self.vel_des_
      yaw_rate_error = -1*(self.yaw_rate_ - self.yaw_rate_des_)

      # Calculate New Input
      velDot = np.matmul(self.A_vel_ - np.matmul(self.B_vel_ ,self.K_vel_),vel_error)
      yaw_rate_dot = self.K_yaw_rate_ * yaw_rate_error

      # TODO: Update State using Integration (ODE45)
      # FOR NOW: Euler Integration
      self.vel_ += velDot * dt
      self.pos_ += self.vel_ * dt
      self.yaw_rate_ += yaw_rate_dot * dt
      # self.yaw_rate_ = self.yaw_rate_des_
      self.yaw_ += self.yaw_rate_ * dt
      self.yaw_ = (self.yaw_ + np.pi) % (2 * np.pi) - np.pi
      self.acc_ = velDot

    else: # We are in position control
      # calculate errors
      pos_error = np.zeros(6)
      pos_error[:3] = self.pos_ - self.pos_des_
      yaw_error = self.yaw_ - self.yaw_des_


      # Calculate New Input
      posDot = np.matmul(self.A_pos_ - np.matmul(self.B_pos_,self.K_pos_), pos_error)
      yawDot = self.K_yaw_ * yaw_error

      if np.linalg.norm(posDot) > self.v_max:
        posDot *= self.v_max/np.linalg.norm(posDot)

      # TODO: Update State using Integration (ODE45)
      # FOR NOW: Euler Integration
      self.vel_ += posDot[-3:] * dt
      self.pos_ += self.vel_ * dt
      self.yaw_rate_ += yawDot * dt
      self.yaw_ += self.yaw_rate_ * dt
      self.acc_ = posDot[-3:]
      

    self.publishData()

    

# /dji_sdk/drone_task_control 
  def DroneTaskControl(self, req):

    # Prepare Parameters
    self.is_vel_ctrl_ = True
    self.yaw_rate_des_ = 0
    v_max_copy = self.v_max
    self.is_vel_ctrl_ = True
    self.yaw_des_ = self.yaw_
    is_takeoff = req.task ==4
    if req.task == 4: 
      des_altitude = rospy.get_param('takeoff_altitude')
    else: 
      des_altitude = rospy.get_param('landing_altitude')

    # Return to above origin
    if req.task == 1:
      self.pos_des_ = [0, 0, des_altitude]
      self.ascend_descend_subroutine()
    
    # Ascend/Descend
    self.pos_des_ = [self.pos_[0], self.pos_[1], des_altitude]
    self.ascend_descend_subroutine(is_takeoff = is_takeoff)

    self.v_max = v_max_copy
    return SimDroneTaskControlResponse(True,4,4,3)


  def ascend_descend_subroutine(self, is_takeoff = False):

    # Set control paramaters for takeoff
    if is_takeoff:
      self.v_max = rospy.get_param('takeoff_vel_max')
      k_pos_error = rospy.get_param('takeoff_pos_error_k')

    # Set control parameters for landing
    if not is_takeoff:
      self.v_max = rospy.get_param('landing_vel_max')
      k_pos_error = rospy.get_param('landing_pos_error_k')

    # Perform flight control
    while np.linalg.norm(self.pos_des_ - self.pos_) > 0.2 or np.linalg.norm(self.vel_) > 0.1:
      # Calculate velocity signal using p control
      self.vel_des_ = k_pos_error * (self.pos_des_ - self.pos_)
      self.yaw_rate_des_ = 0

      # Normalize velocity command
      if np.linalg.norm(self.vel_des_) > self.v_max:
        self.vel_des_ *= self.v_max/np.linalg.norm(self.vel_des_)

      self.performMotion()


if __name__ == '__main__': 
  try:
    rospy.init_node('dji_sdk_sim')

    
    sim = DJI_simulator()

    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass