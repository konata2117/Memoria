#!/usr/bin/env python
# -*- coding: utf-8 -*-
# import the necessary packages
import numpy as np
import argparse
import cv2
from planner import Planner
from doc import PLAN
from parseo import Parseo 
import time
import baxter_interface 
import roslib
import rospy
import cv_bridge
from sensor_msgs.msg import Image
import math
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import tf
import sys
from PIL import Image as ima
from baxter_qr import Camara
rospy.init_node('camarita2',anonymous=True)

foto = None

xx = 0.55 #0.5
yy = 0.48 #0.6
zz = 0.05 #-0.10
roll = -math.pi #Rotacion x
pitch = 0.0 #Rotacion y 
yaw = 0.0       #Rotacion z4


dist = 0.0

pose_i = [xx, yy, zz,roll,pitch, yaw]
pose = [xx,yy,zz,roll, pitch, yaw]
gripper = baxter_interface.Gripper("left")
limb_interface = baxter_interface.Limb('left')
# calibrate the gripper
gripper.calibrate()

# Parametros de camara

cam_calibracion = 0.0025            # 0.0025 pixeles por metro a 1 metro de distancia. Factor de correccion 0.025
  #-0.02            -0.15
#resolution      = 1
width           = 960             # 1280 640  960
height          = 600               # 800  400  600
#The above step is to set the Resolution of the Video. The default is 640x480.
# This examp8le works with a Resolution of 640x480.


###########################################################################
def get_distance(limb):
    if limb == "left":
        dist = baxter_interface.analog_io.AnalogIO('left_hand_range').state()
    elif limb == "right":
        dist = baxter_interface.analog_io.AnalogIO('right_hand_range').state()
    else:
        sys.exit("ERROR - get_distance - Invalid limb")

    if dist > 65000:
        sys.exit("ERROR - get_distance - no distance found")
    print (float(dist / 1000.0))
    return float(dist / 1000.0)

############################################################################3

def callback(msg):
    global foto
    foto = cv_bridge.CvBridge().imgmsg_to_cv2(msg)

#############################################################################3

def send_image(image):  #Shows an image on Baxter's screen
    img = cv2.imread(image) #Reads an image
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8") #Makes the opencv-ros bridge, converts an image to msg
    pub.publish(msg) #Shows the image on Baxter's head screen
    rospy.sleep(0.1)

###############################################################################

def mensaje_matriz_a_pose(T, frame):
    t = PoseStamped()
    t.header.frame_id = frame
    t.header.stamp = rospy.Time.now()
    translacion = tf.transformations.translation_from_matrix(T)
    orientacion = tf.transformations.quaternion_from_matrix(T)
    t.pose.position.x = translacion[0]
    t.pose.position.y = translacion[1]
    t.pose.position.z = translacion[2]
    t.pose.orientation.x = orientacion[0]
    t.pose.orientation.y = orientacion[1]
    t.pose.orientation.z = orientacion[2]
    t.pose.orientation.w = orientacion[3]        
    return t

#############################################################################
def pixel_to_baxter1(px, dist):
 	cam_x_offset = 0
 	cam_y_offset = 0
 	if px[1] >= 300 and px[1] < 500 and px[0] < 200:
    		cam_x_offset = -0.05
    		cam_y_offset = 0.09
    	if px[1] >= 300 and px[1] < 500 and px[0] >= 200 and px[0] < 500:
    		cam_x_offset = -0.13
    		cam_y_offset = 0.09
    	if px[1] > 300 and px[1] < 500 and px[0] >= 500:
    		cam_x_offset= -0.06
    		cam_y_offset = -0.09
    	if px[1] >= 500 and px[1] < 700 and px[0] >= 200 and px[0] < 500:
    		cam_x_offset= -0.03
    		cam_y_offset = 0.09
    	if px[1] >= 700 and px[1] < 850 and px[0] < 200:
	    	cam_x_offset = -0.03
    		cam_y_offset = -0.12
    	if px[1] >= 200 and px[1] < 300 and px[0] >= 500:   
    		cam_x_offset = -0.06
    		cam_y_offset = -0.09
 	print "px[1]", px[1]
    	print "px[0]", px[0]
    	#print "pose_i: ",pose_i
    	x = ((px[1]  - (height / 2)) * cam_calibracion * dist) + pose_i[0] + cam_x_offset

    	y = ((px[0] - (width / 2)) * cam_calibracion * dist)\
         + pose_i[1] + cam_y_offset

    	'''
    	x = ((px[0] - (width / 2)) * cam_calibracion * dist) + pose[0] + cam_x_offset
    	y = ((px[1] - (height / 2)) * cam_calibracion * dist) + pose[1] + cam_y_offset
    	'''
    	#print x , y
    	return (x, y)

def pixel_to_baxter(px, dist):
    #cam_x_offset    = -0.05   # Correccion de camara por los gripper, 0.04 / -0.015 0.045 -0.045 0.05 -0.05
    #cam_y_offset    = 0.08          #-0.15 0.12 -0.12 0.05 0.08
  
    
    '''
    if (px[0] > 0  and px[0] <= 100 and px[1] <= 200):
        cam_y_offset=cam_y_offset + 0.25
        cam_x_offset=cam_x_offset + 0.02
    if (px[0] > 100  and px[0] <= 200 and px[1] <= 200):
        cam_y_offset=cam_y_offset + 0.20
        cam_x_offset=cam_x_offset -0.01
    if (px[0] > 200  and px[0] <= 300 and px[1] <= 200):
        cam_y_offset=cam_y_offset + 0.20
        cam_x_offset=cam_x_offset + 0.02
    
    if (px[0] > 300 and px[0]<= 400 and px[1] <= 200):
        cam_y_offset=cam_y_offset + 0.18
        cam_x_offset=cam_x_offset +0.02
    
    if (px[0] > 400 and px[0]<= 500 and px[1] <= 200):
        cam_y_offset=cam_y_offset + 0.11
        cam_x_offset=cam_x_offset + 0.05    
    

    if (px[0] > 500 and px[0] <= 550 and px[1] <= 200):
        cam_y_offset=cam_y_offset + 0.06
        cam_x_offset=cam_x_offset + 0.02
    
    if (px[0] >= 550 and px[0] <= 600 and px[1] <= 200):
        cam_y_offset=cam_y_offset + 0.07
        cam_x_offset=cam_x_offset + 0.02

    if (px[0] > 600 and px[0]<= 620 and px[1] <= 200):
        cam_y_offset=cam_y_offset + 0.05
        cam_x_offset=cam_x_offset 
    
    if (px[0] > 620 and px[0] <= 700 and px[1] <= 200):
        cam_y_offset=cam_y_offset + 0.06 #0.08
        cam_x_offset=cam_x_offset + 0.01

    
    if (px[0] > 700 and px[0]<= 750 and px[1] <= 200):      
        cam_y_offset=cam_y_offset + 0.04 #revisar para mañana
        cam_x_offset=cam_x_offset + 0.04 
    if (px[0] > 750 and px[0]<= 800 and px[1] <= 200):
        cam_x_offset = cam_x_offset 
        cam_y_offset = cam_y_offset +0.01
    if (px[0]> 800 and px[0] < 850 and px[1] <= 200):
        cam_x_offset=cam_x_offset + 0.04
        cam_y_offset=cam_y_offset +0.06
    if (px[0]> 850 and px[1] <= 200 ):
        cam_x_offset=cam_x_offset + 0.05
        cam_y_offset=cam_y_offset - 0.04
    
        
    if (px[0] > 0  and px[0] <= 100 and px[1] > 200):
        cam_y_offset=cam_y_offset + 0.25
        cam_x_offset=cam_x_offset - 0.01
    if (px[0] > 100  and px[0] <= 200 and px[1] > 200):
        cam_y_offset=cam_y_offset + 0.24
        cam_x_offset=cam_x_offset + 0.02
    if (px[0] > 200  and px[0] <= 300 and px[1] > 200):
        cam_y_offset=cam_y_offset + 0.20
        cam_x_offset=cam_x_offset - 0.02
    if (px[0] > 300 and px[0]<= 400 and px[1] > 200):
        cam_y_offset=cam_y_offset + 0.17
        cam_x_offset=cam_x_offset - 0.05
    if (px[0] > 400 and px[0]<= 500 and px[1] > 200):
        cam_y_offset=cam_y_offset + 0.21
        cam_x_offset=cam_x_offset +0.012 
    if (px[0] > 500 and px[0] <= 550 and px[1] > 200):
        cam_y_offset=cam_y_offset + 0.07
        cam_x_offset=cam_x_offset - 0.05
    if (px[0] >= 550 and px[0] <= 600 and px[1] > 200):
        cam_y_offset=cam_y_offset + 0.06
        cam_x_offset=cam_x_offset -0.12
    if (px[0] > 600 and px[0]<= 620 and px[1] > 200):
        cam_y_offset=cam_y_offset + 0.05
        cam_x_offset=cam_x_offset - 0.01
    if (px[0] > 620 and px[0] <= 700 and px[1] > 200):
        cam_y_offset=cam_y_offset + 0.06 #0.08
        cam_x_offset=cam_x_offset - 0.02
    if (px[0] > 700 and px[0]<= 750 and px[1] > 200):       
        cam_y_offset=cam_y_offset + 0.02 #revisar para mañana
        cam_x_offset=cam_x_offset - 0.03
    if (px[0] > 750 and px[0]<= 800 and px[1] > 200):
        cam_x_offset = cam_x_offset- 0.02
        cam_y_offset = cam_y_offset + 0.04
    
    if (px[0]> 800 and px[0] < 850 and px[1] > 200):
        cam_x_offset=cam_x_offset - 0.06
        cam_y_offset=cam_y_offset + 0.02
    if (px[0]> 850 and px[1] > 200 ):
        cam_x_offset=cam_x_offset + 0.02
        cam_y_offset=cam_y_offset + 0.05
    ''' 
    cam_x_offset = 0
    cam_y_offset = 0
    '''
    if px[1] > 100 and px[0] < 200: 
    	cam_x_offset = -0.03
    	cam_y_offset = 0.11
    '''
    if px[1] >= 300 and px[1] < 500 and px[0] < 200:
    	cam_x_offset = -0.05
    	cam_y_offset = 0.08
    if px[1] >= 300 and px[1] < 500 and px[0] >= 200 and px[0] < 500:
    	cam_x_offset = -0.18
    	cam_y_offset = 0.10
    if px[1] > 300 and px[1] < 500 and px[0] >= 500:
    	cam_x_offset= -0.17
    	cam_y_offset = -0.11
    if px[1] >= 500 and px[1] < 700 and px[0] >= 200 and px[0] < 500:
    	cam_x_offset= -0.03
    	cam_y_offset = 0.08
    if px[1] >= 700 and px[1] < 850 and px[0] < 200:
    	cam_x_offset = -0.03
    	cam_y_offset = -0.12
    if px[1] >= 200 and px[1] < 300 and px[0] >= 500:   
    	cam_x_offset = -0.06
    	cam_y_offset = -0.09
    '''
    if px[1] > 150  and px[1] < 300 and px[0] > 400:
    	cam_x_offset = -0.03
    	cam_y_offset = 0.10
 	'''  
    if px[1] >= 200 and px[1] < 300 and px[0] < 200:
    	cam_x_offset = -0.05
    	cam_y_offset = 0.11
    if px[1] >= 200 and px[1] < 300 and px[0] >= 200  and px[0] < 500:
 		cam_x_offset = -0.04
 		cam_y_offset = 0.09
    '''
    if px[1]> 170 and px[1] < 200 and px[0] > 400:
    	cam_x_offset = -0.03
    	cam_y_offset = 0.02
    
    if  px[0] > 500:
    	cam_x_offset = -0.04
    	cam_y_offset = -0.05
    '''
    if px[1] > 100 and px[1] < 150 and px[0] < 200:
        cam_x_offset = -0.04
        cam_y_offset = 0.11
    if (px[1] > 0 and px[1] < 100 and px[0] < 200):
    	cam_x_offset = -0.04
    	cam_y_offset = 0.11
    if px[1] >= 100 and px[1] < 150 and px[0] >= 200 and px[0] < 500:
    	cam_x_offset = -0.04
    	cam_y_offset = 0.09
    if (px[1] > 0 and px[1] < 100 and px[0] >= 200 and px[0]< 500) or (px[1] >= 150 and px[1] < 200 and px[0] >= 200 and px[0] < 500):
    	cam_x_offset = -0.04
    	cam_y_offset = 0.10
    if px[1] >= 100 and px[1] < 200 and px[0] >= 500:
    	cam_x_offset = -0.04
    	cam_y_offset = -0.11
    if px[1] > 0 and px[1] < 100 and px[0] >= 500:
    	cam_x_offset = -0.04
    	cam_y_offset = -0.11
    if px[1] >= 150 and	 px[1] < 200 and px[0] < 200:
        cam_x_offset = -0.05
        cam_y_offset = 0.11
        '''
    if px[1] > 0 and px[0] <100 :
    	cam_x_offset = -0.05
    	cam_y_offset = -0.12
    '''
    #if px[1] > 0 and px[0] > 400:

    print "px[1]", px[1]
    print "px[0]", px[0]
    #print "pose_i: ",pose_i
    x = ((px[1]  - (height / 2)) * cam_calibracion * dist) + pose_i[0] + cam_x_offset

    y = ((px[0] - (width / 2)) * cam_calibracion * dist)\
         + pose_i[1] + cam_y_offset

    '''
    x = ((px[0] - (width / 2)) * cam_calibracion * dist) + pose[0] + cam_x_offset
    y = ((px[1] - (height / 2)) * cam_calibracion * dist) + pose[1] + cam_y_offset
    '''
    #print x , y
    return (x, y)
##################################################################

def send_image(image):  #Shows an image on Baxter's screen
        pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size = 10)
        img = cv2.imread(image) #Reads an image
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8") #Makes the opencv-ros bridge, converts an image to msg
        pub.publish(msg) #Shows the image on Baxter's head screen
        rospy.sleep(0.1)

####################################################################

def mover_baxter(source_frame, trans, rot):
    nombre_servicio = '/ExternalTools/'+ 'left' +'/PositionKinematicsNode/IKService'
    servicio_ik = rospy.ServiceProxy(nombre_servicio,SolvePositionIK)
    frame = source_frame   

    # Promedio de velocidad del brazo
    limb_interface.set_joint_position_speed(0.5)

    matrix = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(trans),
        tf.transformations.euler_matrix(rot[0],rot[1],rot[2])
        )
        
    rospy.wait_for_service(nombre_servicio,10)
    ik_mensaje = SolvePositionIKRequest()
    ik_mensaje.pose_stamp.append(mensaje_matriz_a_pose(matrix, frame))

    try:
        respuesta = servicio_ik(ik_mensaje)
    except:
        print "Movimiento no ejecutado"
    print respuesta.isValid[0]

    if respuesta.isValid[0] == True:
        movimiento =  dict(zip(respuesta.joints[0].name, respuesta.joints[0].position))
        limb_interface.move_to_joint_positions(movimiento)
    else:
        print "Movimiento no ejecutado"
    

#mover_baxter('base',[xx,yy,zz],[-math.pi,0,0])


i = 0
################################################################
#{'left_w0': -0.4463884092746554, 'left_w1': 1.0879758738077296, 'left_w2': -0.5138835639416136, 'left_e0': -0.43373306777460074, 'left_e1': 1.91824297525071, 'left_s0': 1.7019516841588667, 'left_s1': -0.33018936459233533}
#{'left_s0': 1.6689710972193301, 'left_s1': -0.28033498898605935,'left_e0': -0.4851214241687621, 'left_e1': 1.7648448964621686,'left_w0': -0.4778350154263064, 'left_w1': 1.1362962686261202, 'left_w2': -0.5925000793207411 }
#s0, s1, e0, e1, w0, w1,w2

def get_angles():
		#Shows all the joint angles
	angl = limb_interface.joint_angles() 
	#print angl
	return angl
angles2 ={ 'left_s0': 1.672039058795101, 'left_s1': -0.0023009711818281204, 'left_e0': -0.25464081078897866, 'left_e1': 1.9159420040688817,'left_w0': -0.6653641667452982, 'left_w1': 1.12210694633818, 'left_w2': -0.05445631796993219}
angles = { 'left_s0': 1.4200827143849217, 'left_s1': -0.12808739578843203,'left_e0': -0.07401457301547121, 'left_e1': 1.8875633594930017, 'left_w0': -0.813776807973212, 'left_w1': 1.2440584189750705, 'left_w2': 0.002684466378799474}

angles1 = { 'left_s0': 1.7019516841588667, 'left_s1': -0.33018936459233533, 'left_e0': -0.43373306777460074, 'left_e1': 1.91824297525071,'left_w0': -0.4463884092746554, 'left_w1': 1.0879758738077296, 'left_w2': -0.5138835639416136}
def move_angles(limb, angle):
		limb.set_joint_position_speed(1)
		limb.move_to_joint_positions(angle)
################################################################

def movimiento():
    move_angles(limb_interface,angles1)
    move_angles(limb_interface,angles2)
    angl = get_angles()
    r = open("doc.txt", 'r')
    mensaje = r.read()
    liss = mensaje
    blos = liss.split("\n")
    r.close()
    
    for i in range(len(blos)):
        if blos[i] == "":
            blos.pop(i)
    while not rospy.is_shutdown():  # Capture frame-by-frame
        print "blos: ", blos        
        t=0.16
        capt= Camara('/dev/video', 1)
        circles= capt.capture()
        print "circles " ,circles
        frame=cv2.imread("qrdetectado.jpg")
       # print "bloque, circles", len(circles), len(blos)
        s = 0
       
        bloques_encima = []
        '''
        print angles1['left_s0']
        print angles1['left_s1']
        print angles1['left_e0']
        print angles1['left_e1']
        print angles1['left_w0']
        print angles1['left_w1']
        print angles1['left_w2']
        '''
        #if (angles1['left_s0'] < 1.8019516841588667) :
        	#print "angulos"
        ang= get_angles()
        while circles and len(circles) == len(blos) and (1 < ang['left_s0'] < 1.8019516841588667):
        	#print "if circles encima"
        	if s == len(blos) or len(blos) == 1:
        		break
        	#if len(blos) % 2 == 0:
        	for i in range(len(blos)):
        			if i == len(blos) - 1:
        				if abs(circles[blos[i]][0] - circles[blos[0]][0]) < 50 and len(blos) != 2:
        					if (circles[blos[i]][0] > circles[blos[0]][0]):
        			 			bloques_encima.append(blos[i])
        			 			#print "bloque encima blos - 1 mayor", bloques_encima
        			 			s = s + 1

        			 			#print s
        			 			break
        					else:
	        			 		bloques_encima.append(blos[0])
	        			 		#print "bloque encima blos - 1 menor", bloques_encima
        			 			s = s + 1
        			 			break
        					s = s + 1
        				else: 
	        				s = s + 1
	        				break
        			if abs(circles[blos[i]][0] - circles[blos[i+1]][0]) < 50 and len(blos) != 2:
	        			if (circles[blos[i]][1] > circles[blos[i + 1]][1]):
        			 		bloques_encima.append(blos[i + 1])
        			 		#print "bloque encima blos mayor", bloques_encima
        				else:
	        			 	bloques_encima.append(blos[i ])
	        			 	#print "bloque encima blos - 1 menor", bloques_encima
        				s = s + 1
        			else: 
	        			s = s + 1
 
	        		#print "i , s " ,i , s
	      
        bloques_encima = set(bloques_encima)
        bloques_encima = list(bloques_encima)
        
        print "bloque encima ", bloques_encima
        #if len(bloques_encima) == 0 :
        #	mover_baxter('base',[xx - 0.02 ,yy + 0.05,zz],[-math.pi,0,0])
        #if len(bloques_encima) != 0 :
        #	move_angles(limb_interface, angles1)
       # if bloques_encima > 0 or (bloques_encima == 0 and ang1['left_s0'] > 1):
        #	move_angles(limb_interface, angles1)
        while len(bloques_encima) > 0:
        	bloque_busqueda = []
        	move_angles(limb_interface, angles1)
        	mover_baxter('base',[xx,yy - 0.04,zz],[-math.pi,0,0])
        #	print " angulos, " ,get_angles()
        	circles2= capt.capture()
        	print "circles2 " , circles2
        #	print "circles, bloque encima", len(circles), len(bloques_encima)
        	#print "blosques encima circles: ", circles
        	while circles2 and len(bloques_encima) > 0:
        		cir = circles2.keys()
        		t = False
        		for i in range(len(bloques_encima)):
        			for j in range(len(cir)):
        				if cir[j] == bloques_encima[i]:
        					t = True  

        		if t == False:
        			break
        		else:
        			pu=circles2[bloques_encima[0]]
        			print "pu ", pu
        			bloque_busqueda.append(bloques_encima[0])

        			puss = pixel_to_baxter((pu[0], pu[1] ),dist)
                	mover_baxter('base',[puss[0] + 0.03,puss[1],0.0],[-math.pi,0,0])
                	rospy.sleep(0.5)
               		mover_baxter('base',[puss[0], puss[1], -0.10],[-math.pi,0 ,0])
               		send_image("do.jpg")
               		mover_baxter('base',[puss[0],puss[1],-0.13],[-math.pi,0,0])
               		rospy.sleep(0.5)
               		gripper.close()
               		rospy.sleep(0.5)
           			#cosa= gripper.set_moving_force()
               		print "cosa: " ,gripper.force()
               		if gripper.force()==0:
	                		print 'no hay nada en el gripper'
                			send_image("que.jpg")
                			mover_baxter('base',[xx,yy - 0.04 ,zz],[-math.pi,0,0])
                			gripper.open()
                		#mover_baxter('base',[xx,yy,zz],[-math.pi,0,0])
	                		break
               		else:
                            rospy.sleep(0.3)
                            mover_baxter('base',[puss[0],puss[1],0.0],[-math.pi,0,0])
                            rospy.sleep(1)
                            if puss[0] < 400:
                                mover_baxter('base',[puss[0] - 0.13,puss[1],0.0],[-math.pi,0,0])
                                rospy.sleep(0.5)
                                mover_baxter('base',[puss[0] - 0.13,puss[1],-0.10],[-math.pi,0,0])
                                rospy.sleep(0.5)
                                mover_baxter('base',[puss[0] - 0.13,puss[1],-0.16],[-math.pi,0,0])
                                gripper.open()
                                rospy.sleep(0.5)
                                mover_baxter('base',[puss[0]- 0.13, puss[1],-0.10], [-math.pi,0,0])
                                rospy.sleep(0.5)
                                mover_baxter('base',[puss[0] - 0.13,puss[1],0.0],[-math.pi,0,0])
                            if puss[0] > 400:
                                mover_baxter('base',[puss[0] - 0.18,puss[1],0.0],[math.pi,0,0])
                                rospy.sleep(0.5)
                                mover_baxter('base',[puss[0] - 0.18,puss[1],-0.10],[-math.pi,0,0])
                                rospy.sleep(0.5)
                                mover_baxter('base',[puss[0] - 0.18,puss[1],-0.16],[math.pi,0,0])
                                gripper.open()
                                rospy.sleep(0.5)
                                mover_baxter('base',[puss[0] - 0.18,puss[1],-0.10],[-math.pi,0,0])
                                rospy.sleep(0.5)
                                mover_baxter('base',[puss[0] - 0.18,puss[1],0.0],[math.pi,0,0])
                            send_image("pop.jpg")
                            rospy.sleep(0.5)
                            mover_baxter('base',[xx, yy -0.04, zz],[-math.pi,0,0])
                            #mover_baxter('base',[0.4,-0.3,t],[-math.pi,0,0])
                            #pose_i = [pun[0], pun[1], z, roll, pitch, yaw]
                            #pose = [pun[0], pun[1], z, roll, pitch, yaw]
                            print "pase por aca"
                            print "circles blo, bloque_encima ", circles2[bloques_encima[0]], bloques_encima[0]
                            del circles2[bloques_encima[0]]
                            bloques_encima.pop(0)
                            print "circles blo, bloque_encima ", circles2, bloques_encima

        
        circles1= capt.capture()
        ang1 = get_angles()

        while circles1 and len(bloques_encima) == 0 and ang1['left_s0'] < 1:
        	  
        	  if len(circles1) != len(blos):
        	  	break
        	  if len(circles1) == len(blos):
        	  	rr=open("inicio.txt",'w')
        	  	for i in range(len(blos)):
        	  		if circles1[blos[i]] != None:
        	  			rr.write(blos[i] + "\n")
        	  	rr.close()
        	  	planer= PLAN()
        	  	plas = planer.planes("doc.txt","p8.pddl","inicio.txt")
        	  	print "plas: ", plas
        	  	send_image("feliz.jpg")
        	  	print "planer"
        	  	plans = Planner()
        	  	planns = plans.solve("blocksworld.pddl","p8.pddl")
        	  	print "planns ", planns
        	  	rs = open("elplan.txt",'w')
        	  	if planns:
        	  		rs.write('plan: ' + "\n")
        	  		for act in planns:
        	  			rs.write(str(act))
        	  	else:
        	  		rs.write('No hay un plan')
        	  	rs.close()
        	  	pars = Parseo()
        	  	acciones, parametros = pars.parseo("elplan.txt")
        	  	print "accion y parametro " ,acciones, parametros
        	  	break
        	  else:
        	  	break

        cir ={}
        for i in range(len(blos)):
            cir[blos[i]] = [1,1]
        bloque_busqueda1 = []
        busquedas2 = []
        while circles1 and len(bloques_encima) == 0 and ang1['left_s0'] < 1:
            
            #poner el error del circle
            if len(blos) == 0 :
                break
            if len(circles1) != len(blos):
                break
            
    
            #print "acciones", len(acciones['action'])
            punto=circles1[blos[0]]
            print "primer bloque"
            print "punto: ", punto[0],punto[1]
            del circles1[blos[0]]
            print "punto: ", punto[0],punto[1]
            pun=pixel_to_baxter((punto[0], punto[1] ),dist)

            bloque_busqueda1.append(blos[0])
            #centro1= pixel_to_baxter((480,300),dist)
            #print "centro1 ", centro1
            rospy.sleep(0.5)
            print "pun: ",[pun]
            mover_baxter('base',[pun[0],pun[1],0.0],[-math.pi,0,0])
            
            send_image("do.jpg")
            mover_baxter ('base', [pun[0], pun[1], - 0.10], [-math.pi,0,0])
            rospy.sleep(0.5)
            mover_baxter('base',[pun[0],pun[1],-0.18],[-math.pi,0,0])
            rospy.sleep(0.5)
            gripper.close()
            rospy.sleep(0.5)
            print "cosa: " ,gripper.force() 
            if gripper.force()==0:
                print 'no hay nada en el gripper'
                send_image("que.jpg")
                mover_baxter('base',[xx ,yy - 0.04,zz],[-math.pi,0,0])
                gripper.open()
                break        
            else:
                    rospy.sleep(0.3)
                    mover_baxter('base',[pun[0],pun[1],0.0],[-math.pi,0,0])
                    rospy.sleep(1)
                    mover_baxter('base',[0.57,0.1,0.0],[-math.pi,0,0])
                    rospy.sleep(1)
                    mover_baxter('base', [0.57,0.1, -(t / 2)], [-math.pi, 0, 0])
                    rospy.sleep(0.5)
                    mover_baxter('base',[0.57,0.1,-t],[-math.pi,0,0])
                    gripper.open()
                    rospy.sleep(0.2)
                    mover_baxter('base', [0.57,0.1, -(t/2)],[-math.pi, 0 ,0])
                    rospy.sleep(0.5)
                    mover_baxter('base',[0.57,0.1,t],[-math.pi,0,0])
                    send_image("pop.jpg")
                    rospy.sleep(0.5)
                    mover_baxter('base',[xx ,yy - 0.04,zz],[-math.pi,0,0])
                    rospy.sleep(0.5)
                    #mover_baxter('base',[xx, yy , zz],[-math.pi,0,0])
              
                    print "pase por aca"
            	
            cir[blos[0]][0] = 0.57
            cir[blos[0]][1] = 0.1
            blos.pop(0)
            t=t-0.04
            
            print "circ", cir
            
            for i in range(len(acciones['action'])):
            	print "action", acciones['action'][i]
            	print "parametro[i] ",parametros[i]
                print "blos 0 ",blos[0]
                print "parametro i 0", parametros[i][0].upper()
                if acciones['action'][i] == 'pickup':
                    print "en el pickup"
                    punto=circles1[parametros[i][0].upper()] #cambiar la iteracion aca de 2 y 1 listas

                    #cir[parametros[i].upper()] = []
                    #cir[parametros[i].upper()].append(punto[0])
                    #cir[parametros[i].upper()].append(punto[1])
                    #print "punto: ", punto[0],punto[1]
                    print "circles1 ",circles1
                    del circles1[blos[0]]    
                    
                    print "punto: ", punto[0],punto[1]
                    pun=pixel_to_baxter((punto[0], punto[1] ),dist)
                    rospy.sleep(0.5)
                    print "pun: ",[pun]
                    mover_baxter('base',[pun[0],pun[1],0.0],[-math.pi,0,0])
                    send_image("do.jpg")
                    mover_baxter('base',[pun[0], pun[1], -0.10],[-math.pi,0,0])
                    rospy.sleep(0.5)
                    mover_baxter('base',[pun[0],pun[1],-0.18],[-math.pi,0,0])
                    rospy.sleep(0.5)
                    gripper.close()
                    rospy.sleep(0.5)
                    mover_baxter('base', [pun[0], pun[1], -0.10], [-math.pi,0,0])
                    rospy.sleep(0.5)
                    mover_baxter('base',[pun[0],pun[1],0.0],[-math.pi,0,0])
                    rospy.sleep(0.5)
                    print "cosa: " ,gripper.force()
                    if gripper.force()==0:
                        print 'no hay nada en el gripper'
                        send_image("que.jpg")
                        mover_baxter('base',[xx ,yy - 0.04,zz],[-math.pi,0,0])
                        gripper.open()
                        break
                    else:
                        continue

            	if acciones['action'][i] == 'stack':
                        print "en el stack"
                    	punto1=cir[parametros[i][1].upper()]
                    	mover_baxter('base',[punto1[0],punto1[1],0.0],[-math.pi,0,0])
                    	rospy.sleep(0.5)
                    	mover_baxter('base', [punto1[0], punto1[1], -(t / 2)],[-math.pi,0,0])
                    	rospy.sleep(0.5)
                    	mover_baxter('base',[punto1[0],punto1[1],-t],[-math.pi,0,0])
                    	rospy.sleep(0.5)
                    	gripper.open()
                    	rospy.sleep(0.5)
                    	mover_baxter('base',[punto1[0],punto1[1],-( t/2)],[-math.pi,0,0])
                    	rospy.sleep(0.5)
                    	mover_baxter('base',[punto1[0],punto1[1],t],[-math.pi,0,0])
                    	rospy.sleep(0.5)
                    	mover_baxter('base',[punto1[0] + 0.03,punto1[1],t],[-math.pi,0,0])
                    	send_image("pop.jpg")
                    	rospy.sleep(0.5)
                    	mover_baxter('base',[xx,yy - 0.04,zz],[-math.pi,0,0])
                    	print "pase por aca"
                        cir[parametros[i][0].upper()][0]= punto1[0]
                        cir[parametros[i][0].upper()][1] = punto1[1]
                blos.pop(0)
               	i = i +  1
               	t=t-0.04

       
               
                	#if acciones['action'][i] == 'stack':
                	#    print "wait"
                	#del acciones['action'][i]
                	#del parametros[i]
         
def main():
    
    movimiento()
if __name__=='__main__':
    main() 
    #pose_i = [pun[0], pun[1], zz, roll, pitch, ya  w]
    #pose = [pun[0], pun[1], zz, roll, pitch, yaw]