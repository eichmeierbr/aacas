#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, UInt8
from sensor_msgs.msg import NavSatFix
from dji_sdk.srv import SetLocalPosRef


class serviceCaller:

    def __init__(self):

        self.gps_health = 0

        set_pos_name = 'dji_sdk/set_local_pos_ref'
        rospy.wait_for_service(set_pos_name)
        self.set_pos_service = rospy.ServiceProxy(set_pos_name, SetLocalPosRef)


        rospy.Subscriber('/dji_sdk/gps_health', UInt8, self.gps_health_cb)


    def gps_health_cb(self, msg):
        self.gps_health = msg.data


if __name__ == '__main__': 
  try:
    rospy.init_node('local_position_setter')
    rate = rospy.Rate(10) # 10hz
    caller = serviceCaller()

    ########### Wait for GPS Lock  #############
    # rospy.loginfo("Getting Satellite Fix")
    # while caller.gps_health < rospy.get_param('min_gps_health'):
        # rate.sleep()
    # rospy.loginfo("Obtained Satellite Fix")


    set_pos_name = 'dji_sdk/set_local_pos_ref'
    rospy.wait_for_service(set_pos_name)
    set_pos_service = rospy.ServiceProxy(set_pos_name, SetLocalPosRef)

    resp = False

    while resp == False:
        resp = set_pos_service()
        rate.sleep()

    rospy.loginfo('Local Position Set')
    
  except rospy.ROSInterruptException:
      pass


