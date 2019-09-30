
'''
from PIL import Image
from pytesser import *
 
image_file = 'Frame_22.tif'
im = Image.open(image_file)
text = image_to_string(im)
text = image_file_to_string(image_file)
text = image_file_to_string(image_file, graceful_errors=True)
print "=====output=======\n"
print text
'''

import cv2
from PIL import Image
import pytesseract

image_file = 'ver1.jpg'
#im = Image.open(image_file)
image = cv2.imread(image_file)
gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
gray = cv2.medianBlur(gray,3)

text = pytesseract.image_to_string(gray,lang='spa', config = '-psm 9')
#text = image_file_to_string(image_file)
#text = image_file_to_string(image_file, graceful_errors=True)
print "=====output=======\n"
print text


'''
try:
    from PIL import Image
except ImportError:
    import Image
import pytesseract

def ocr_core(filename):
    """
    This function will handle the core OCR processing of images.
    """
    #print pytesseract.image_to_osd(Image.open(filename))
    
    #text = pytesseract.image_to_string(Image.open(filename))  # We'll use Pillow's Image class to open the image and pytesseract to detect the string in the image
    #return text

ocr_core("crop.jpg")
'''
'''
from PIL import Image
import pytesseract
import cv2
from scipy import ndimage
import numpy as np
image  = cv2.imread("azul1.jpg")
kernel = np.ones((2,2), np.uint8)
if (preprocess=="thresh"):
    # https://docs.opencv.org/trunk/d7/d4d/tutorial_py_thresholding.html
    gray = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]

# make a check to see if median blurring should be done to remove noise
elif (preprocess=="blur"):
    gray = cv2.medianBlur(gray,3)
#image = ndimage.rotate(image, 12)#12
#imagen = cv2.erode(image,kernel,iterations= 1)
#gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
#gray = cv2.medianBlur(gray,3)
#cv2.imwrite("cropgray.jpg",imagen)
text = pytesseract.image_to_string(image, lang='spa', config = '-psm 9')#
print text
 
# show the output images
'''
cv2.imshow("Image", image)
cv2.imshow("Output", gray)
while(1):
    tecla = cv2.waitKey(5) & 0xFF
    if tecla == 27:
        break
 
cv2.destroyAllWindows()