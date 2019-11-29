# -*- coding: utf-8 -*-
import cv2
import sys
import imutils
from pyzbar import pyzbar
import argparse
import time
import math
#import rospy
#import baxter_interface
#from sensor_msgs.msg import Image
#import cv_bridge
import numpy as np 

#from limb import Limb 
from circulos import hough
from qrs import qr

foto = None

class Camara:
    BRIGHTNESS = 0.0
    width = 960
    height = 600
    def __init__(self, dir, device):
        self.cap = dir
        self.device = device
    
    def release(self):
        self.cap.release()
    
    def capture(self):
       #t = 0   
       while True:
        self.cap = cv2.VideoCapture(self.device) 
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,Camara.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, Camara.height)    
        #self.cap.set(3, 1280) # set the resolution 

        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)
        rval, frame = self.cap.read()
        if not rval or frame is None:
            print 'None'
            continue
        foto = frame
        if cv2.waitKey(27) >= 0:
            break
        #circles = hough(frame)


        #cv2.imwrite("framecircles.jpg",frame)
        #if circles is not None :
         #   print (frame)
            #cv2.destroyAllWindows()
            #break
        #else: 
         #   print "cirles", circles

        qro= qr(foto)
        cv2.imwrite("qrdetectado.jpg", foto)
        #t = t + 1
        #if t == 5:
         #   break
        
        
        self.release()   
        return qro
        #return circles
def main():
        capturer = Camara('/dev/video', 1)
        capturer.capture()

if __name__ == '__main__':
    while (1):
        main()
        print "hello"
    