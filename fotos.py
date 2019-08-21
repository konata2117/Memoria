#!/usr/bin/env python
# -*- coding: utf-8 -*-
# import the necessary packages
import numpy as np
import argparse
import cv2
import cv2.cv as cv 
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

rospy.init_node('camarita2',anonymous=True)
cap = baxter_interface.CameraController('left_hand_camera') 
cap.open()
cap.resolution=(960,600)
#cap.resolution = cap.MODES[0]
foto = None
def QtoE(): #Quaternion to Euler. Converts Quaternion angles(x, y, z, w) into Euler angles (x, y ,z) and prints them
		euler = tf.transformations.euler_from_quaternion(limb_interface.endpoint_pose()['orientation'])
		print ("Arm positions and Quaternion angles")
		print (limb_interface.endpoint_pose())
		print ("Arm Euler angles: ", euler)
# Pose inicial 47 x27 cm
xx = 0.33 #0.5
yy = 0.48 #0.6
zz = -0.08
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
		cam_y_offset=cam_y_offset + 0.15
		cam_x_offset=cam_x_offset +0.05
	if (px[0] > 200  and px[0] <= 300):
		cam_y_offset=cam_y_offset + 0.12
		cam_x_offset=cam_x_offset + 0.05
	if (px[0] > 300 and px[0]<= 400):
		cam_y_offset=cam_y_offset + 0.11
		cam_x_offset=cam_x_offset + 0.07
	if (px[0] > 400 and px[0]<= 500):
		cam_y_offset=cam_y_offset + 0.11
		cam_x_offset=cam_x_offset + 0.09	
	

	if (px[0] > 500 and px[0] <= 700):
		cam_y_offset=cam_y_offset + 0.09
		cam_x_offset=cam_x_offset + 0.08
	'''
	if (px[0] >= 570 and px[0] <= 600):
		cam_y_offset=cam_y_offset + 0.09
		cam_x_offset=cam_x_offset + 0.06

	if (px[0] > 600 and px[0]<= 620):
		cam_y_offset=cam_y_offset + 0.09
		cam_x_offset=cam_x_offset + 0.06
	
	if (px[0] > 620 and px[0] <= 700):
		cam_y_offset=cam_y_offset + 0.09 #0.08
		cam_x_offset=cam_x_offset + 0.06
	'''
			
	if (px[0] > 700 and px[0]<= 800):		
		cam_y_offset=cam_y_offset + 0.08
		cam_x_offset=cam_x_offset +0.09
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


def transformacion(frame):
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #tranformacion de la imagen a escla de grises
	dimensiones =gray.shape
	kernel = np.ones((3,3),np.uint8)
	gray = cv2.GaussianBlur(gray,(5,5),0)#para eliminar el ruido de la imagen
	gray = cv2.medianBlur(gray,5) #reduce el ruido de sal
	gray = cv2.erode(gray,kernel,iterations = 1)#erosiona la imagen
	gray = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,3.5)
	#canny = cv2.Canny(gray, 40, 80)
	#cv2.imshow("GR",gray)
	#detectar circulos en la imagen
	circles =cv2.HoughCircles(gray,cv.CV_HOUGH_GRADIENT,1, 100, param1=40, param2=40,minRadius=45, maxRadius=160)
	#circles = cv2.HoughCircles(gray, cv.CV_HOUGH_GRADIENT, 1, 200, param1=40, param2=45, minRadius=20, maxRadius=140)
	return circles 

def transformacion1(x,y,frame):
	
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #tranformacion de la imagen a escla de grises
	dimensiones =gray.shape
	kernel = np.ones((3,3),np.uint8)
	gray = cv2.GaussianBlur(gray,(5,5),0)#para eliminar el ruido de la imagen
	gray = cv2.medianBlur(gray,5) #reduce el ruido de sal
	gray = cv2.erode(gray,kernel,iterations = 1)#erosiona la imagen
	gray = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,3.5)
	#canny = cv2.Canny(gray, 40, 80)
	#cv2.imshow("GR",gray)
	#detectar circulos en la imagen
	
	
	circles =cv2.HoughCircles(gray,cv.CV_HOUGH_GRADIENT,1, 100, param1=40, param2=40,minRadius=45, maxRadius=160)
	#circles = cv2.HoughCircles(gray, cv.CV_HOUGH_GRADIENT, 1, 200, param1=40, param2=45, minRadius=20, maxRadius=140)
	return circles 


def dibujarcirculo(circles,frame):
	print "circles" , circles
	if circles is not None:

		# convert the (x, y) coordinates and radius of the circles to integers
		circles2 = np.round(circles[0, :]).astype("int")
		# loop over the (x, y) coordinayates and radius of the circles
		for (x, y, r) in circles2:	
			cv2.circle(frame, (x, y), r, (0, 255, 0), 4) #dibujar circulo periferico
			cv2.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1) #dibujar rectangulo en el centro
			cv2.rectangle(frame , (470,290), (490,310), (0,128,255),-1)
			#cv2.rectangle(frame , (0,0), (980,600), (0,128,255),2)
			cv2.rectangle(frame , (200,0), (210,0), (0,128,255),2)
			cv2.rectangle(frame , (400,0), (410,0), (0,128,255),2)
			cv2.rectangle(frame , (600,0), (610,0), (0,128,255),2)
			cv2.rectangle(frame , (800,0), (810,0), (0,128,255),2)
#right_camera_callback(data)
rospy.Subscriber('/cameras/left_hand_camera/image', Image, callback)

#while not rospy.is_shutdown():
#QtoE()
dist1 = 0
dist2= 0

def CalcularDistancia(circo):
	cir = []
	while circo:
		circle= circo.pop()
		dist1 = math.sqrt((480-circle[0])**2+(300-circle[1])**2)
		cir.append((dist1, circle[0],circle[1], circle[2]))
		print "Distancia: ", dist1
	return cir

def OrdenarCirculo(cir):
	cir.sort()
	pp =[]
	while cir:
		p= cir.pop()
		pp.append((p[3], p[1],p[2]))
		pp.sort()
	return pp

t=0.16
angles = limb_interface.joint_angles() 
def get_angles():	#Shows all the joint angles
	print angles
get_angles()
#{'left_w0': 1.9331992879325928, 'left_w1': -1.4101118392636667, 'left_w2': -3.0591411862404865, 'left_e0': -0.022626216621309852, 'left_e1': 1.5784662307340906, 'left_s0': 1.0833739314440736, 'left_s1': 0.13805827090968723}
limb_interface.move_to_joint_positions(angles) 
#mover_baxter('base',[xx ,yy,zz],[-math.pi,0,0])
dist = get_distance("left")
circle	=[]
while not rospy.is_shutdown():	# Capture frame-by-frame
	while np.all(foto) == None:
	#	print "Ahorita no joven"
		continue
	frame=foto
	cv2.imwrite("F7.jpg",frame)
	print "Foto tomada"
	#circles=transformacion(frame)
	#dibujarcirculo(circles,frame)
	#copia_cir=circles
	#print type(copia_cir)

	#print "Circulos", len(circles)
	#rospy.sleep(2)
	'''
	circles=np.round(circles[0,:].astype("int"))


	
	#cv2.imshow("Frame",frame)
	#cv2.waitKey(0)
	#send_image("Frame.jpg")
	#acercarse a la torre

	#circles1= []
	#print type(circles1) 
	
	print "Dimension: ", circles.ndim
	#for i in range(circles.ndim):
	#	circles1.append(circles[i])
	#print circles1
	#x1=0.61195328392783256
	#y1=-0.6652463368344972
	#deposito2=[x=0.31195328392783256, y=0.6652463368344972]
	circles1=list(circles)

	print "circles1",circles1
	print "Tamaño Circle1: ",len(circles1)
	e=0.02
	tamano=len(circles1)
	circo = []
	for i in range(len(circles1)):
		circo.append(circles1[i])
	circ=CalcularDistancia(circo)
	dd= OrdenarCirculo(circ)

	for i in range(len(circles1)):
		circles1.pop()
	for i in range(len(dd)):
		circles1.append(dd[i])
	print circles1
	y=0.05
	while circles1:
		
		punto=circles1.pop()
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
	
	


			
		'''	
	#pose_i = [pun[0], pun[1], zz, roll, pitch, yaw]
	#pose = [pun[0], pun[1], zz, roll, pitch, yaw]
