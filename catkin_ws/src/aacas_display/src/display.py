#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Joy
from lidar_process.msg import tracked_obj_arr
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped, PointStamped, Point, Vector3, Quaternion
# from scipy.spatial.transform import Rotation as R
# from dji_m600_sim.srv import SimDroneTaskControl
# from dji_sdk.srv import DroneTaskControl, SDKControlAuthority, SetLocalPosRef
# from aacas_detection.srv import QueryDetections
# from lidar_process.msg import tracked_obj, tracked_obj_arr
from PyQt5 import QtWidgets, QtCore, QtGui
# sudo apt-get install python-pyqt5
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg
# pip install pyqtgraph
import sys



class Objects:
    def __init__(self, pos = np.zeros(3), vel=np.zeros(3), dist = np.inf, id=0):
        self.id = id
        self.position = pos
        self.velocity = vel
        self.distance = dist
        self.last_orbit_change_ = rospy.Time.now() - rospy.Duration(secs=1000)
        self.orbit = -1
  

class vectDisplay:

    def __init__(self, waypoints = [[0,0,0]]):
        

        # state Information
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.vel_ctrl = np.zeros(4) #global coordinates
        self.yaw = 0
        self.yaw_rate = 0
        self.quat = Quaternion()
        self.pos_pt = Point()
        self.is_safe = True
        self.detections = []

        # Publisher Information
        #vel_ctrl_pub_name = rospy.get_param('vel_ctrl_sub_name')
        #self.vel_ctrl_pub_ = rospy.Publisher(vel_ctrl_pub_name, Joy, queue_size=10)

        #pos_ctrl_pub_name = rospy.get_param('pos_ctrl_sub_name')
        #self.pos_ctrl_pub_ = rospy.Publisher(pos_ctrl_pub_name, Joy, queue_size=10)

        # Subscriber Information

        rospy.Subscriber(rospy.get_param('obstacle_trajectory_topic'), tracked_obj_arr, self.updateDetections, queue_size=1) 
        rospy.Subscriber(rospy.get_param('position_pub_name'), PointStamped,      self.position_callback, queue_size=1)
        rospy.Subscriber(rospy.get_param('velocity_pub_name'), Vector3Stamped,    self.velocity_callback, queue_size=1)
        rospy.Subscriber(rospy.get_param('attitude_pub_name'), QuaternionStamped, self.attitude_callback, queue_size=1)
        # rospy.Subscriber(rospy.get_param('vel_ctrl_sub_name'), Joy, self.attitude_callback, queue_size=1)

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


    def updateDetections(self, msg):
        # in_detections = self.query_detections_service_(vehicle_position=self.pos_pt, attitude=self.quat)
        in_detections = msg.tracked_obj_arr
        vect_lst = []
        for obj in in_detections:
            # newObj = Objects()
            
            newObj = [obj.point.x,obj.point.y]
            # print(obj)
            # print(newObj)
            # newObj.velocity = Point(0,0,0)
            # newObj.id = obj.object_id
            # newObj.distance = np.linalg.norm([obj.point.x - self.pos[0], obj.point.y - self.pos[1], obj.point.z - self.pos[2]])
            vect_lst.append(newObj)
            # print(len(self.detection))
        # print(self.detections,"\n")
        # print(len(vect_lst))
        self.detections = vect_lst
        # print(self.detections[0])



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

        
def smooth(onew,oold):
    osmooth = (onew*3+oold*1)/4
    
    return osmooth

def smooth_obj(a,old_detect,ratio):
    ratio = 0.6
    
    # print(a)
    
    for i in range(min(len(a),len(old_detect))):
        if len(a[i])==len(old_detect[i]):
            big = np.ones(len(a))*ratio
            small = np.ones(len(a))*(1-ratio)
        # if a[i,0]<old_detect[i,0]+1 and a[i,0]>old_detect[i,0]-1:
            a[i] = np.multiply(a[i],ratio)+np.multiply(old_detect[i],(1-ratio))
            print(a[i])
        # print("hi")
    #     smoothx = (newx*big+oldx*small)/100

    # if newy.shape==oldy.shape:
    #     smoothy = (newy*big+oldy*small)/100
        # print("")
    
    # return osmooth
    return a

def make_points(field):
    a = field.yaw

    mat = np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1]])
    x3 = [field.vel[0],field.vel[1],0]
    corrected = np.dot(x3,mat)
    # print(corrected)
    x = [0,-corrected[1]*2]
    y = [0,corrected[0]*2]



    

    return x,y

def get_obj(field):
    a = field.detections
    our_x = field.pos[0]
    our_y = field.pos[1]
    old_detect = a


    x = [0,0]
    y = [0,0]
    for i in range(len(a)):
        ith_detection = a[i]
        x.append((ith_detection[0]-our_x)/1)
        y.append((ith_detection[1]-our_y)/1)

    o = field.yaw
    mat = np.array([[np.cos(o),-np.sin(o),0],[np.sin(o),np.cos(o),0],[0,0,1]])
    x3 = np.append(np.append(np.array(x),np.array(y)),np.ones(len(x)))
    x3 = x3.reshape(3,len(x))
    corrected = np.dot(x3.T,mat)
    xnew = -corrected[:,1]
    ynew = corrected[:,0]

    
    return xnew,ynew,o,old_detect

def update_obj(field,oold,old_detect):

    a = field.detections
    our_x = field.pos[0]
    our_y = field.pos[1]
    # a = smooth_obj(a,old_detect,.9)
    # a0,a1 = smooth_obj(a[:,0],a[:,1],oldx,oldy,.9)
    # oldx = a[:,0]
    # oldy = a[:,1]
    old_detect = a


    x = [0,0]
    y = [0,0]
    for i in range(len(a)):
        ith_detection = a[i]
        x.append((ith_detection[0]-our_x)/1)
        y.append((ith_detection[1]-our_y)/1)

    o = smooth(field.yaw,oold)
    mat = np.array([[np.cos(o),-np.sin(o),0],[np.sin(o),np.cos(o),0],[0,0,1]])
    x3 = np.append(np.append(np.array(x),np.array(y)),np.ones(len(x)))
    x3 = x3.reshape(3,len(x))
    corrected = np.dot(x3.T,mat)
    xnew = -corrected[:,1]
    ynew = corrected[:,0]

    
    return xnew,ynew,o,old_detect

def update_points(field):
    

    a = field.yaw

    mat = np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1]])
    x3 = [field.vel[0],field.vel[1],1]
    corrected = np.dot(x3,mat)
    # print(corrected)
    x = [0,-corrected[1]*20]
    y = [0,corrected[0]*20]



    return x,y


def make_circle(w,h,r=10,step=5):

    circlex = []
    circley = []

    for i in range(-w,w,step):
        for j in range(-h,h,step):
            if ((i/float(w))**2+(j/float(h))**2<=1):
                circlex.append(i/float(w)*r)
                circley.append(j/float(h)*r)      
            
    # print(circlex)
    return circlex,circley

def fov(degree=45):

    x = 10*np.sin(degree/float(180)*np.pi)
    y = 10*np.cos(degree/float(180)*np.pi)
    return [0,x],[0,y]




class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.title = "Awesome GUI"
        self.top = 500
        self.left = 1000
        self.width = 700
        self.height = 700
        self.multiplier = 0.8
        self.step = 1


        self.InitWindow()

    def InitWindow(self):
        self.graphWidget = pg.PlotWidget()
        self.setCentralWidget(self.graphWidget)
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)
        self.constant_background()
        # 
        # self.objects_display()
        self.fov()
        self.velocity_vector()
        # self.constant_foreground()

        self.graphWidget.setXRange(-10, 10, padding=0.05)
        self.graphWidget.setYRange(-10, 10, padding=0.05)

        self.graphWidget.addLegend()
        self.graphWidget.showGrid(x=True, y=True)
        self.graphWidget.setBackground('w')

        # self.graphWidget.getPlotItem().hideAxis('bottom')
        # self.graphWidget.getPlotItem().hideAxis('left')

        self.show()
        
    def constant_background(self):

        # gray outline
        circlex,circley = make_circle(self.width,self.height,r=10,step=self.step)
        pen = pg.mkPen(color=(220, 220, 220), width=self.step, style=QtCore.Qt.SolidLine)
        self.gray_circle =  self.graphWidget.plot(circlex, circley, name="10m Safety Radius", pen=pen)

        # white background
        circlex,circley = make_circle(self.width,self.height,r=9.5,step=self.step)
        pen = pg.mkPen(color=(255, 255, 255), width=self.step, style=QtCore.Qt.SolidLine)
        self.white_circle =  self.graphWidget.plot(circlex, circley, pen=pen)

    def fov(self):
        # FOV of camera
        x,y = fov(degree=30)
        pen = pg.mkPen(color=(220, 220, 220), width=self.step, style=QtCore.Qt.SolidLine)
        self.white_circle =  self.graphWidget.plot(x, y, name="FOV", pen=pen)
        x,y = fov(degree=-30)
        pen = pg.mkPen(color=(220, 220, 220), width=self.step, style=QtCore.Qt.SolidLine)
        self.white_circle =  self.graphWidget.plot(x, y, name="FOV", pen=pen)

    def constant_foreground(self):

        circlex,circley = make_circle(self.width,self.height,r=1.5,step=self.step)
        pen = pg.mkPen(color=(255, 255, 255), width=self.step, style=QtCore.Qt.SolidLine)
        self.white1 =  self.graphWidget.plot(circlex, circley, name="Robo body", pen=pen)

        # body outline
        circlex,circley = make_circle(self.width,self.height,r=.5,step=self.step*10)
        pen = pg.mkPen(color=(0, 0, 0), width=self.step, style=QtCore.Qt.SolidLine)
        self.robo =  self.graphWidget.plot(circlex, circley, name="Robo body", pen=pen)

        circlex,circley = make_circle(self.width,self.height,r=.5,step=self.step*10)
        pen = pg.mkPen(color=(0, 0, 0), width=self.step, style=QtCore.Qt.SolidLine)
        self.robo1 =  self.graphWidget.plot(np.add(circlex,.6), np.add(circley,.6), name="Robo body", pen=pen)

        circlex,circley = make_circle(self.width,self.height,r=.5,step=self.step*10)
        pen = pg.mkPen(color=(0, 0, 0), width=self.step, style=QtCore.Qt.SolidLine)
        self.robo2 =  self.graphWidget.plot(np.add(circlex,.6), np.add(circley,-.6), name="Robo body", pen=pen)

        circlex,circley = make_circle(self.width,self.height,r=.5,step=self.step*10)
        pen = pg.mkPen(color=(0, 0, 0), width=self.step, style=QtCore.Qt.SolidLine)
        self.robo3 =  self.graphWidget.plot(np.add(circlex,-.6), np.add(circley,.6), name="Robo body", pen=pen)

        circlex,circley = make_circle(self.width,self.height,r=.5,step=self.step*10)
        pen = pg.mkPen(color=(0, 0, 0), width=self.step, style=QtCore.Qt.SolidLine)
        self.robo4 =  self.graphWidget.plot(np.add(circlex,-.6), np.add(circley,-.6), name="Robo body", pen=pen)

        
    def velocity_vector(self):

        
        self.field = vectDisplay()
    
    

        # rospy.sleep(2)

        rospy.loginfo("DISPLAY")
        self.x,self.y = make_points(self.field)
        self.ox,self.oy,self.yaw,self.old_detect = get_obj(self.field)

        pen = pg.mkPen(color=(255, 255, 255), width=0, style=QtCore.Qt.SolidLine)
        self.data_line2 =  self.graphWidget.plot(self.ox,self.oy, name="Flight Direction",symbolSize=30, pen=pen,symbol='o')  #, symbol='<', symbolSize=30, symbolBrush=('b'))


        pen = pg.mkPen(color=(255, 0, 0), width=int(self.width/100), style=QtCore.Qt.SolidLine)
        self.data_line1 =  self.graphWidget.plot(self.x, self.y, name="Flight Direction", pen=pen)  #, symbol='<', symbolSize=30, symbolBrush=('b'))

        

     

        # needed to update plot efficiently
        self.timer = QtCore.QTimer()
        self.timer.setInterval(10)
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()

    # def objects_display(self):

        
    #     self.field = vectDisplay()
    
    

    #     # rospy.sleep(2)

    #     rospy.loginfo("DISPLAY")
    #     self.ox,self.oy,self.yaw = get_obj(self.field)

    #     pen = pg.mkPen(color=(255, 255, 255), width=int(self.width/100), style=QtCore.Qt.SolidLine)
    #     self.data_line2 =  self.graphWidget.plot(self.ox,self.oy, name="Flight Direction",symbolSize=30, pen=pen,symbol='o')  #, symbol='<', symbolSize=30, symbolBrush=('b'))

        

    #     # needed to update plot efficiently
    #     self.timer = QtCore.QTimer()
    #     self.timer.setInterval(10)
    #     self.timer.timeout.connect(self.update_obj_data)
    #     self.timer.start()

    def update_plot_data(self):
        self.x,self.y = update_points(self.field)
        self.ox,self.oy,self.yaw,self.old_detect = update_obj(self.field,self.yaw,self.old_detect)
        # self.x = [0,self.field.vel[0]]
        # self.y = [0,self.field.vel[1]]
        self.data_line2.setData(self.ox, self.oy)
        self.data_line1.setData(self.x, self.y)
        
    
    # def update_obj_data(self):
    #     self.ox,self.oy,self.yaw = update_obj(self.field,self.yaw)
    #     # self.x = [0,self.field.vel[0]]
    #     # self.y = [0,self.field.vel[1]]
    #     self.data_line2.setData(self.ox, self.oy)









if __name__ == '__main__': 
  try:
    rospy.init_node('vectDisplay')

    # Launch Node
    

    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    sys.exit(app.exec_())


    # while not rospy.is_shutdown():
        # print(field.vel)
        # make_points(field.vel)
        #field.pos = numpy array
	#do stuff
   


    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass

