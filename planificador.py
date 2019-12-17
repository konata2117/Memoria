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

def pixel_to_baxter(px, dist):
    cam_x_offset    = -0.05   # Correccion de camara por los gripper, 0.04 / -0.015 0.045 -0.045 0.05
    cam_y_offset    = -0.12             #-0.15 0.12
    '''
    if (px[0] >1000):
    
        cam_y_offset= cam_y_offset + 0.05
    '''
    
    if (px[0] > 0  and px[0] <= 100 and px[1] <= 200):
        cam_y_offset=cam_y_offset + 0.25
        cam_x_offset=cam_x_offset + 0.02
    if (px[0] > 100  and px[0] <= 200 and px[1] <= 200):
        cam_y_offset=cam_y_offset + 0.22
        cam_x_offset=cam_x_offset +0.04
    if (px[0] > 200  and px[0] <= 300 and px[1] <= 200):
        cam_y_offset=cam_y_offset + 0.21
        cam_x_offset=cam_x_offset + 0.04
    
    if (px[0] > 300 and px[0]<= 400 and px[1] <= 200):
        cam_y_offset=cam_y_offset + 0.19
        cam_x_offset=cam_x_offset +0.05
    
    if (px[0] > 400 and px[0]<= 500 and px[1] <= 200):
        cam_y_offset=cam_y_offset + 0.11
        cam_x_offset=cam_x_offset + 0.05    
    

    if (px[0] > 500 and px[0] <= 550 and px[1] <= 200):
        cam_y_offset=cam_y_offset + 0.09
        cam_x_offset=cam_x_offset -0.02
    
    if (px[0] >= 550 and px[0] <= 600 and px[1] <= 200):
        cam_y_offset=cam_y_offset + 0.06
        cam_x_offset=cam_x_offset + 0.02

    if (px[0] > 600 and px[0]<= 620 and px[1] <= 200):
        cam_y_offset=cam_y_offset + 0.06
        cam_x_offset=cam_x_offset - 0.03
    
    if (px[0] > 620 and px[0] <= 700 and px[1] <= 200):
        cam_y_offset=cam_y_offset + 0.05 #0.08
        cam_x_offset=cam_x_offset + 0.05

    
    if (px[0] > 700 and px[0]<= 750 and px[1] <= 200):      
        cam_y_offset=cam_y_offset + 0.03 #revisar para mañana
        cam_x_offset=cam_x_offset + 0.05 
    if (px[0] > 750 and px[0]<= 800 and px[1] <= 200):
        cam_x_offset = cam_x_offset 
        cam_y_offset = cam_y_offset +0.05
    if (px[0]> 800 and px[0] < 850 and px[1] <= 200):
        cam_x_offset=cam_x_offset - 0.03
        cam_y_offset=cam_y_offset -0.01
    if (px[0]> 850 and px[1] <= 200 ):
        cam_x_offset=cam_x_offset + 0.05
        cam_y_offset=cam_y_offset - 0.04
    
        
    if (px[0] > 0  and px[0] <= 100 and px[1] > 200):
        cam_y_offset=cam_y_offset + 0.25
        cam_x_offset=cam_x_offset - 0.01
    if (px[0] > 100  and px[0] <= 200 and px[1] > 200):
        cam_y_offset=cam_y_offset + 0.24
        cam_x_offset=cam_x_offset -0.02
    if (px[0] > 200  and px[0] <= 300 and px[1] > 200):
        cam_y_offset=cam_y_offset + 0.18
        cam_x_offset=cam_x_offset - 0.01
    if (px[0] > 300 and px[0]<= 400 and px[1] > 200):
        cam_y_offset=cam_y_offset + 0.13
        cam_x_offset=cam_x_offset 
    if (px[0] > 400 and px[0]<= 500 and px[1] > 200):
        cam_y_offset=cam_y_offset + 0.12
        cam_x_offset=cam_x_offset -0.03 
    if (px[0] > 500 and px[0] <= 550 and px[1] > 200):
        cam_y_offset=cam_y_offset + 0.07
        cam_x_offset=cam_x_offset + 0.03
    if (px[0] >= 550 and px[0] <= 600 and px[1] > 200):
        cam_y_offset=cam_y_offset + 0.07
        cam_x_offset=cam_x_offset -0.07
    if (px[0] > 600 and px[0]<= 620 and px[1] > 200):
        cam_y_offset=cam_y_offset + 0.05
        cam_x_offset=cam_x_offset - 0.05
    if (px[0] > 620 and px[0] <= 700 and px[1] > 200):
        cam_y_offset=cam_y_offset + 0.04 #0.08
        cam_x_offset=cam_x_offset -0.04
    if (px[0] > 700 and px[0]<= 750 and px[1] > 200):       
        cam_y_offset=cam_y_offset + 0.02 #revisar para mañana
        cam_x_offset=cam_x_offset - 0.05
    if (px[0] > 750 and px[0]<= 800 and px[1] > 200):
        cam_x_offset = cam_x_offset + 0.05
        cam_y_offset = cam_y_offset - 0.01
    
    if (px[0]> 800 and px[0] < 850 and px[1] > 200):
        cam_x_offset=cam_x_offset - 0.07
        cam_y_offset=cam_y_offset -0.03
    if (px[0]> 850 and px[1] > 200 ):
        cam_x_offset=cam_x_offset + 0.05
        cam_y_offset=cam_y_offset - 0.04
        
    print "px[0]", px[0]
    print "px[1]", px[1]
    print "pose_i: ",pose_i
    x = ((px[1]  - (height / 2)) * cam_calibracion * dist) + pose_i[0] + cam_x_offset

    y = ((px[0] - (width / 2)) * cam_calibracion * dist)\
         + pose_i[1] + cam_y_offset

    '''
    x = ((px[0] - (width / 2)) * cam_calibracion * dist) + pose[0] + cam_x_offset
    y = ((px[1] - (height / 2)) * cam_calibracion * dist) + pose[1] + cam_y_offset
    '''
    print x , y
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
angles = { 'left_s0': 1.7019516841588667, 'left_s1': -0.33018936459233533, 'left_e0': -0.43373306777460074, 'left_e1': 1.91824297525071,'left_w0': -0.4463884092746554, 'left_w1': 1.0879758738077296, 'left_w2': -0.5138835639416136}
def move_angles(limb, angle):
		limb.set_joint_position_speed(1)
		limb.move_to_joint_positions(angle)
################################################################

def movimiento():
    move_angles(limb_interface,angles)
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
        print circles
        frame=cv2.imread("qrdetectado.jpg")
        print "bloque, circles", len(circles), len(blos)
        s = 0
        bloques_encima = []
        while circles and len(circles) == len(blos):
        	if s == len(blos) or len(blos) == 1:
        		break
        	if len(blos) % 2 == 0:
        		for i in range(len(blos)):
        			if i == len(blos) - 1:
        				if abs(circles[blos[i]][0] - circles[blos[0]][0]) < 100:
        					if (circles[blos[i]][0] > circles[blos[0]][0]):
        			 			bloques_encima.append(blos[i])
        			 			print "bloque encima blos - 1 mayor", bloques_encima
        			 			s = s + 1
        			 			#print s
        			 			break
        					else:
	        			 		bloques_encima.append(blos[0])
	        			 		print "bloque encima blos - 1 menor", bloques_encima
        			 			s = s + 1
        			 			break
        					s = s + 1
        				else: 
	        				s = s + 1
	        				break
        			if abs(circles[blos[i]][0] - circles[blos[i+1]][0]) < 100:
	        			if (circles[blos[i]][1] > circles[blos[i + 1]][1]):
        			 		bloques_encima.append(blos[i + 1])
        			 		print "bloque encima blos mayor", bloques_encima
        				else:
	        			 	bloques_encima.append(blos[i ])
	        			 	print "bloque encima blos - 1 menor", bloques_encima
        				s = s + 1
        			else: 
	        			s = s + 1
 
	        		print "i , s " ,i , s
        bloques_encima = set(bloques_encima)
        bloques_encima = list(bloques_encima)
        print "bloque encima ", bloques_encima
        while len(bloques_encima) > 0:
        	mover_baxter('base',[xx ,yy + 0.07,zz],[-math.pi,0,0])
        	circles= capt.capture()
        	print "circles, bloque encima", len(circles), len(bloques_encima)
        	print "blosques encima circles: ", circles
        	while circles:
        		if len(circles) != len(bloques_encima):
        			break
        		else:
        			pu=circles[bloques_encima[0]]
        			print "pu ", pu
        		
        			puss = pixel_to_baxter((pu[0], pu[1] ),dist)
                	mover_baxter('base',[puss[0],puss[1],0.0],[-math.pi,0,0])
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
                		mover_baxter('base',[xx,yy,zz],[-math.pi,0,0])
                		gripper.open()
                	#mover_baxter('base',[xx,yy,zz],[-math.pi,0,0])
                		break
                	else:
                		rospy.sleep(0.3)
                		mover_baxter('base',[puss[0],puss[1],0.0],[-math.pi,0,0])
                		rospy.sleep(1)
                		mover_baxter('base',[puss[0] - 0.12,puss[1],0.0],[-math.pi,0,0])
                		rospy.sleep(1)
                		mover_baxter('base',[puss[0] - 0.12,puss[1],-0.18],[-math.pi,0,0])
                    	gripper.open()
                    	rospy.sleep(0.2)
                    	mover_baxter('base',[puss[0] - 0.12,puss[1],0.18],[-math.pi,0,0])
                    	send_image("pop.jpg")
                    	rospy.sleep(0.5)
                    	mover_baxter('base',[xx,yy,zz],[-math.pi,0,0])
                		#mover_baxter('base',[0.4,-0.3,t],[-math.pi,0,0])
                		#pose_i = [pun[0], pun[1], z, roll, pitch, yaw]
                		#pose = [pun[0], pun[1], z, roll, pitch, yaw]
                    	print "pase por aca"
                    	del circles[bloques_encima[0]]
                    	bloques_encima.pop(0)
        #circles= capt.capture()
        while circles:
        	  if len(circles) == len(blos):
        	  	rr=open("inicio.txt",'w')
        	  	for i in range(len(blos)):
        	  		if circles[blos[i]] != None:
        	  			rr.write(blos[i] + "\n")
        	  	rr.close()
        	  	planer= PLAN()
        	  	plas = planer.planes("doc.txt","p8.pddl","inicio.txt")
        	  	print plas
        	  	send_image("feliz.jpg")
        	  	print "planer"
        	  	plans = Planner()
        	  	planns = plans.solve("blocksworld.pddl","p8.pddl")
        	  	print planns
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
        while circles:
            
            #poner el error del circle
            if len(blos) == 0 :
                break
            if len(circles) != len(blos):
                break
            
            cir = {}
            print len(acciones['action'])

            for i in range(len(acciones['action'])):
                if acciones['action'][i] == 'pickup':
                    punto=circles[parametros[i][i].upper()]

                    cir[parametros[i][i].upper()] = []
                    cir[parametros[i][i].upper()].append(punto[0])
                    cir[parametros[i][i].upper()].append(punto[1])
                    print "punto: ", punto[0],punto[1]
                    print acciones['action'][i]
                    print parametros[1]
                    
        
            #punto=circles[parametros[i][i].upper()]
                    
                    del circles[blos[0]]    
            
                    print "punto: ", punto[0],punto[1]
                    pun=pixel_to_baxter((punto[0], punto[1] ),dist)
                    centro1= pixel_to_baxter((480,300),dist)
            #print punto[2]
            #(puntox,puntoy)=pixel_to_baxter(punto,0.3)
            #print "puntox: ",puntox ,"puntoy: ",puntoy
                    rospy.sleep(0.5)
                    print "pun: ",[pun]
            
                    mover_baxter('base',[pun[0],pun[1],0.0],[-math.pi,0,0])
            #pose_i = [pun[0]-0.05, pun[1]+0.05, zz, roll, pitch, yaw]
            #pose = [pun[0]-0.05, pun[1]+0.05, zz, roll, pitch, yaw]
                    send_image("do.jpg")
                    mover_baxter('base',[pun[0],pun[1],-0.18],[-math.pi,0,0])
                    rospy.sleep(0.5)
                    gripper.close()
                    rospy.sleep(0.5)
            #cosa= gripper.set_moving_force()
                    print "cosa: " ,gripper.force() 

                    if gripper.force()==0:
                        print 'no hay nada en el gripper'
                        send_image("que.jpg")
                        mover_baxter('base',[xx,yy,zz],[-math.pi,0,0])
                        gripper.open()
                        mover_baxter('base',[xx,yy,zz],[-math.pi,0,0])
                        break
            
                    else:
                        rospy.sleep(0.3)
                        mover_baxter('base',[pun[0],pun[1],0.0],[-math.pi,0,0])
                        rospy.sleep(1)
                        mover_baxter('base',[0.47,0.1,0.0],[-math.pi,0,0])
                        rospy.sleep(1)
                        mover_baxter('base',[0.47,0.1,-t],[-math.pi,0,0])

            
                        gripper.open()
                        rospy.sleep(0.2)
                        mover_baxter('base',[0.47,0.1,t],[-math.pi,0,0])
                        send_image("pop.jpg")
                        rospy.sleep(0.5)
                        mover_baxter('base',[xx,yy,zz],[-math.pi,0,0])
                #mover_baxter('base',[0.4,-0.3,t],[-math.pi,0,0])
                #pose_i = [pun[0], pun[1], z, roll, pitch, yaw]
                #pose = [pun[0], pun[1], z, roll, pitch, yaw]
                        print "pase por aca"
                #circles[blos[0]]
                blos.pop(0)

                i = i +  1
                t=t-0.05
                if acciones['action'][i] == 'stack':
                    print "wait"
                del acciones['action'][i]
                del parametros[i]
            
def main():
    
    movimiento()
if __name__=='__main__':
    main() 
    #pose_i = [pun[0], pun[1], zz, roll, pitch, yaw]
    #pose = [pun[0], pun[1], zz, roll, pitch, yaw]