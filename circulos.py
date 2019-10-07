import numpy as np
import cv2
import time 

def hough(frame):
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
	#circles =cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1, 100, param1=40, param2=40,minRadius=45, maxRadius=160)
	circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 200, param1=40, param2=45, minRadius=20, maxRadius=140)
	print "circles" , circles

	if circles is not None:# convert the (x, y) coordinates and radius of the circles to integers
		circles2 = np.round(circles[0, :]).astype("int")
		#circles= np.uint16(np.round(circles))
		# loop over the (x, y) coordinayates and radius of the circles
		for (x, y, r) in circles2:	
			cv2.circle(frame, (x, y), r, (0, 255, 0), 4) #dibujar circulo periferico
			cv2.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1) #dibujar rectangulo en el centro
			#cv2.rectangle(frame , (470,290), (490,310), (0,128,255),-1)
			#cv2.rectangle(frame , (0,0), (980,600), (0,128,255),2)
			#cv2.rectangle(frame , (200,0), (210,0), (0,128,255),2)
			#cv2.rectangle(frame , (400,0), (410,0), (0,128,255),2)
			#cv2.rectangle(frame , (600,0), (610,0), (0,128,255),2)
			#cv2.rectangle(frame , (800,0), (810,0), (0,128,255),2)
	cv2.imshow("circulos", frame)
	if cv2.waitKey(27) >= 0:
           pass
	return circles 


def main():
	frame = cv2.imread("ro.jpg")
	circles = hough(frame)
if __name__=='__main__':
	main() 