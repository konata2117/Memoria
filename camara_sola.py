#!/usr/bin/env python

import rospy
import baxter_interface
import roslib
import cv2
#import cv2.cv as cv
import cv_bridge
from sensor_msgs.msg import Image
import numpy as np 


#Iniciar Nodo
rospy.init_node('camarita2', anonymous = True)

camara = baxter_interface.CameraController('left_hand_camera')

camara.open()

camara.resolution          = camara.MODES[0]
#camara.exposure            = -1             # range, 0-100 auto = -1
#camara.gain                = -1             # range, 0-79 auto = -1
#camara.white_balance_blue  = -1             # range 0-4095, auto = -1
#camara.white_balance_green = -1             # range 0-4095, auto = -1
#camara.white_balance_red   = -1             # range 0-4095, auto = -1

foto = None

def callback(msg):
    # Transforma el mensaje a imagen
    global foto
    foto = cv_bridge.CvBridge().imgmsg_to_cv2(msg) #, "bgr8") #bgr8
    # Abre una ventana con la imagen. 'Foto Nueva' es el nombre de la ventana.

rospy.Subscriber('/cameras/left_hand_camera/image', Image , callback)

while not rospy.is_shutdown():
#Capturar un frame
    while np.all(foto) == None:
        #print "hola"
        continue 

    frame = foto
    limb_interface = baxter_interface.Limb('left')
    #Capturar un frame con la camara y guardar sus dimensiones
    frame_gris = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    dimensiones = frame_gris.shape #'dimensiones' sera un array que contendra el alto, el ancho y los canales de la imagen en este orden.
    angles = limb_interface.joint_angles() 
    def get_angles():   #Shows all the joint angles
        print angles
    get_angles()
    #Mostrar la imagen
    cv2.imshow('Imagen', frame)
    cv2.imwrite('Frame_4.jpg',frame)
    #Salir con 'ESC'
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break
 
cv2.destroyAllWindows()










