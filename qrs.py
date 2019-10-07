# -*- coding: utf-8 -*-
from pyzbar import pyzbar
import argparse
import datetime
import imutils
import time
import cv2

def qr(frame):
    # Construir el analizador de argumentos
    ap = argparse.ArgumentParser()
    ap.add_argument("-o", "--output", type=str, default="baxter.csv") 
    args = vars(ap.parse_args())


    print("[INFO] Inicializando variables y archivo .CSV")


    # Abre el archivo CSV de salida para escribir e inicializar
    # códigos de barras (código qr) encontrados hasta ahora
    csv = open(args["output"], "w")
    found = set()

    #   loop sobre el stream
    
        # tomar el cuadro de la secuencia de video y cambiar su tamaño a un ancho máximo de 400 píxeles
        
    frame = imutils.resize(frame, width=960)
    # encuentra los códigos de barras (código qr) en el tablero y decodifica cada uno de los códigos de barras    
    barcodes = pyzbar.decode(frame)
    # loop sobre los códigos de barras detectados
    lis= {}    
    for barcode in barcodes:
        # Extrae la ubicación del cuadro delimitador de código de barras y dibuja
        # el cuadro delimitador que rodea el código de barras en la imagen (en este caso es verde)
        (x, y, w, h) = barcode.rect
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        
        print x, y ,w ,h
       # cv2.rectangle(frame, (round(x/2), round(y/2)), (round((x+w )/ 2), round((y+h)/ 2)) , (0,255,0), 2)
        # los datos del código de barras son un objeto byte, así que si queremos dibujarlos
        # en la imagen de salida, necesitamos convertirla a un string
        barcodeData = barcode.data.decode("utf-8")
        print type(barcodeData) 
        barcodeType = barcode.type
        # dibuja los datos del código de barras y el tipo de código de barras en la imagen
        text = "{}". format (barcodeData)
        text= text.upper()

        print type(text)
        #cv2.putText(frame, text, (cx , cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0,255,0),2)
        cv2.putText(frame, text, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        if barcodeData not in found:
                csv.write("{}\n".format(barcodeData))
                csv.flush()
                found.clear()
                found.add(barcodeData)
        lis[text]= []
        print lis

        # Título de la vetnana
    cv2.imshow("QR_Reader", frame)
    

    # Archivo .csv
    print("[INFO] Finalizando, cerrando el archivo CSV....")
    csv.close()
    return lis
    #cv2.destroyAllWindows()
def main():
    frame = cv2.imread("azul.png")
    qr(frame)
if __name__=='__main__':
    main() 