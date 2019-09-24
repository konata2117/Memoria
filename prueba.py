import cv2
import numpy as np
 
#Iniciamos la camara
imagen  = cv2.imread("azul.jpg")
 

     
#Capturamos una imagen y la convertimos de RGB -> HSV

hsv = cv2.cvtColor(imagen, cv2.COLOR_BGR2HSV)
 #convert it to hsv

for x in range(0, len(hsv)):
    for y in range(0, len(hsv[0])):
        hsv[x, y][2] += 80

hsv = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
#Establecemos el rango de colores que vamos a detectar
#En este caso de verde oscuro a verde-azulado claro
verde_bajos = np.array([49,50,50], dtype=np.uint8)
verde_altos = np.array([80, 255, 255], dtype=np.uint8)#80
#Azules:
azul_bajos = np.array([100,65,75], dtype=np.uint8)
azul_altos = np.array([130, 255, 255], dtype=np.uint8)
#Rojos:
rojo_bajos1 = np.array([0,65,75], dtype=np.uint8)
rojo_altos1 = np.array([12, 255, 255], dtype=np.uint8)
rojo_bajos2 = np.array([240,65,75], dtype=np.uint8)
rojo_altos2 = np.array([256, 255, 255], dtype=np.uint8)
 
#Crear una mascara con solo los pixeles dentro del rango de verdes
mask = cv2.inRange(hsv, verde_bajos, verde_altos)
mask1 = cv2.inRange(hsv, azul_bajos, azul_altos) 
#Encontrar el area de los objetos que detecta la camara
moments = cv2.moments(mask)
mm = cv2.moments(mask1)
area = moments['m00']

 
#Descomentar para ver el area por pantalla
print area
if(area > 2000000):
         
        #Buscamos el centro x, y del objeto
        x = int(moments['m10']/moments['m00'])
        y = int(moments['m01']/moments['m00'])
         
        #Mostramos sus coordenadas por pantalla
        print "x = ", x
        print "y = ", y
 
        #Dibujamos una marca en el centro del objeto
        cv2.rectangle(imagen, (x, y), (x+2, y+2),(0,0,255), 2)
     
     
#Mostramos la imagen original con la marca del centro y
#la mascara
cv2.imshow('mask', mask)
cv2.imshow('mask1',mask1)
cv2.imshow('Camara', imagen)
while(1):
    tecla = cv2.waitKey(5) & 0xFF
    if tecla == 27:
        break

 
cv2.destroyAllWindows()