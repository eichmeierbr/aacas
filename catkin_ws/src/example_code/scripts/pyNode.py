#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
# from PACKAGE_NAME.srv import SERVICE1, SERVICE2, etc


class Example:
    def __init__(self):

        # Initialize variables from the parameter file
        # self.get_param_ = rospy.get_param('~param_name', default='default value')
        self.var1_ = rospy.get_param('pyVar1_default', default=0)
        self.var2_ = rospy.get_param('pyVar2_default',default=0)
        self.response_step = rospy.get_param('py_response_step',default = 2)
        self.service_reset = rospy.get_param('py_service_reset', default = 100)


        self.py_pub_name = rospy.get_param('py_pub_name')
        self.cpp_pub_name = rospy.get_param('cpp_pub_name')


        ## Example Publisher Information
        # self.template_pub_ = rospy.get_param(Topic_Name, DATA_TYPE, queue_size=10)
        self.example_pub_ = rospy.Publisher(self.py_pub_name, Float64, queue_size=10)


        ## Example Subscriber Information
        # rospy.Subscriber(TOPIC_NAME, DATA_TYPE, CALL_BACK, queue_size=10)
        rospy.Subscriber(self.cpp_pub_name, Float64, self.sub_callback, queue_size=10)


        ## Service Server Information
        # self.serv_ = rospy.Service(SERVICE_NAME, SERVICE_TYPE, SERVICE_CALLBACK)



    ## Service Client Information
    # def server_client
    #     rospy.wait_for_service(SERVICE_NAME)
    #     try:

    #         service_input = []

    #         service_var = rospy.ServiceProxy(SERVICE_NAME, SERVICE_TYPE)
    #         service_response = service_var(service_input)
    #         return service_response.data
    #     except rospy.ServiceException, e:
    #         print "Service call failed: %s"%e


    def sub_callback(self, received_data):
        self.var1_ = received_data.data

        # Do things here
        newData = self.var1_ + self.response_step

        # Publish the data
        self.example_pub_.publish(newData)



    def serv_callback(self,req):
        self.var2_ = req.data
        if self.var2_> self.service_reset:
          self.var2_= 0
        return self.var2_ + self.response_step




if __name__ == '__main__': 
  try:
    rospy.init_node('python_node')

    
    ex = Example()

    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass