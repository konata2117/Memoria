#!/usr/bin/env python
# -*- coding: utf-8 -*-
# import the necessary packages
import numpy as np
import argparse
import cv2

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
#cap = baxter_interface.CameraController('left_hand_camera') 
#cap.open()
#cap.resolution=(960,600)
#cap.resolution = cap.MODES[0]


#cap = cv2.VideoCapture(2)

   



#cv2.imshow(cap2)
#cv2.waitKey(0)
foto = None
#def QtoE(): #Quaternion to Euler. Converts Quaternion angles(x, y, z, w) into Euler angles (x, y ,z) and prints them
		#euler = tf.transformations.euler_from_quaternion(limb_interface.endpoint_pose()['orientation'])
		#print ("Arm positions and Quaternion angles")
		#print (limb_interface.endpoint_pose())
		#print ("Arm Euler angles: ", euler)
# Pose inicial 47 x27 cm
xx = 0.5 #0.5
yy = 0.48 #0.6
zz = 0.0 #-0.10
roll = -math.pi	#Rotacion x
pitch = 0.0	#Rotacion y	
yaw = 0.0		#Rotacion z

dist = 0.0

pose_i = [xx, yy, zz,roll,pitch, yaw]
pose = [xx,yy,zz,roll, pitch, yaw]
gripper = baxter_interface.Gripper("left")
limb_interface = baxter_interface.Limb('left')
# calibrate the gripper
gripper.calibrate()

# Parametros de camara

cam_calibracion = 0.0025            # 0.0025 pixeles por metro a 1 metro de distancia. Factor de correccion 0.025
  #-0.02			-0.15
#resolution      = 1
width           = 960             # 1280 640  960
height          = 600               # 800  400  600
#The above step is to set the Resolution of the Video. The default is 640x480.
# This examp8le works with a Resolution of 640x480.



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

def callback(msg):
	global foto
	foto = cv_bridge.CvBridge().imgmsg_to_cv2(msg)

def send_image(image):	#Shows an image on Baxter's screen
	img = cv2.imread(image)	#Reads an image
	msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8") #Makes the opencv-ros bridge, converts an image to msg
	pub.publish(msg) #Shows the image on Baxter's head screen
	rospy.sleep(0.1)

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

def pixel_to_baxter(px, dist):
	cam_x_offset    = -0.05      # Correccion de camara por los gripper, 0.04 / -0.015 0.045 -0.045
	cam_y_offset    = -0.12 			#-0.15
	'''
	if (px[0] >1000):
	
		cam_y_offset= cam_y_offset + 0.05
	'''
	
	if (px[0] > 0  and px[0] <= 100):
		cam_y_offset=cam_y_offset + 0.06
		cam_x_offset=cam_x_offset + 0.08
	if (px[0] > 100  and px[0] <= 200):
		cam_y_offset=cam_y_offset + 0.14
		cam_x_offset=cam_x_offset +0.08
	if (px[0] > 200  and px[0] <= 300):
		cam_y_offset=cam_y_offset + 0.12
		cam_x_offset=cam_x_offset + 0.05
	if (px[0] > 300 and px[0]<= 400):
		cam_y_offset=cam_y_offset + 0.08
		cam_x_offset=cam_x_offset + 0.07
	if (px[0] > 400 and px[0]<= 500):
		cam_y_offset=cam_y_offset + 0.08
		cam_x_offset=cam_x_offset + 0.05	
	

	if (px[0] > 500 and px[0] <= 550):
		cam_y_offset=cam_y_offset + 0.1
		cam_x_offset=cam_x_offset + 0.05
	
	if (px[0] >= 550 and px[0] <= 600):
		cam_y_offset=cam_y_offset + 0.09
		cam_x_offset=cam_x_offset + 0.09

	if (px[0] > 600 and px[0]<= 620):
		cam_y_offset=cam_y_offset + 0.09
		cam_x_offset=cam_x_offset + 0.06
	
	if (px[0] > 620 and px[0] <= 700):
		cam_y_offset=cam_y_offset + 0.09 #0.08
		cam_x_offset=cam_x_offset + 0.06
	'''
	if (px[0] > 750 and px[1] > 500):
		cam_x_offset =cam_x_offset + 0.03
		cam_y_offset = cam_y_offset + 0.03
	'''
	if (px[0] > 700 and px[0]<= 750):		
		cam_y_offset=cam_y_offset + 0.07
		cam_x_offset=cam_x_offset +0.06
	if (px[0]> 800):
		cam_x_offset=cam_x_offset + 0.05
		cam_y_offset=cam_y_offset + 0.05
	
	
	
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

def send_image(image):	#Shows an image on Baxter's screen
		pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size = 10)
		img = cv2.imread(image)	#Reads an image
		msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8") #Makes the opencv-ros bridge, converts an image to msg
		pub.publish(msg) #Shows the image on Baxter's head screen
		rospy.sleep(0.1)
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
	#print respuesta.joints[0].position
	#print respuesta.joints[0].name

        # 3ms wait
#cv.WaitKey(3)




mover_baxter('base',[xx,yy,zz],[-math.pi,0,0])



def movimiento():

	while not rospy.is_shutdown():	# Capture frame-by-frame
		t=0.16
		capt= Camara('/dev/video', 1)
		circles= capt.capture()
	
		frame=cv2.imread("qrdetectado.jpg")
#		cv2.imwrite("Frame_35.jpg",frame)
		#print "imagen 
	

		#copia_cir=circles
		#print type(copia_cir)


		#print "Circulos", len(circles)
		#rospy.sleep(2)
		circles=np.round(circles[0,:].astype("int"))

	
	
		#cv2.imshow("Frame",frame)
		#cv2.waitKey(0)


		#send_image("Frame.jpg")
		#acercarse a la torre

		#circles1= []
		#print type(circles1) 
	
		print "Dimension: ", circles.ndim
		#for i in range(circles.ndim):
		#circles1.append(circles[i])
		#print circles1
		#x1=0.61195328392783256
		#y1=-0.6652463368344972
		#deposito2=[x=0.31195328392783256, y=0.6652463368344972]
		circles1=list(circles)

		print "circles1",circles1
		print "Tamaño Circle1: ",len(circles1)
		e=0.02
		tamano=len(circles1)
		

		i = 0
		while circles1:
		
			punto=circles1.pop()
			#cv2.imshow("frame",frame)
			#cv2.waitKey(0)
			#frame1 = ima.open("./qrdetectado.jpg")
			#cropped = frame1.crop((punto[1]- punto[0] - 100,punto[2]- punto[0] - 100 , punto[0] + 100 + punto[1], 100 +punto[2] + punto[0]))
			#name = str(i) + ".jpg"
			#cropped.show()
			#cropped.save(name)
			#i= i+ 1 
			send_image("feliz.jpg")
			print "tamaño while: ",len(circles1)
			print "punto: ", punto[1],punto[2]
			pun=pixel_to_baxter((punto[1], punto[2] ),dist)
			centro1= pixel_to_baxter((480,300),dist)
			print punto[2]
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
				mover_baxter('base',[pun[0],pun[1],0.0],[-math.pi,0,0])
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
				mover_baxter('base',[pun[0],pun[1],0.0],[-math.pi,0,0])
				#mover_baxter('base',[0.4,-0.3,t],[-math.pi,0,0])
				#pose_i = [pun[0], pun[1], z, roll, pitch, yaw]
				#pose = [pun[0], pun[1], z, roll, pitch, yaw]
				print "pase por aca"
				t=t-0.02


def main():
	
	movimiento()
if __name__=='__main__':
    main() 
	#pose_i = [pun[0], pun[1], zz, roll, pitch, yaw]
	#pose = [pun[0], pun[1], zz, roll, pitch, yaw]