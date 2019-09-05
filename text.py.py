
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
from PIL import Image
import pytesseract

image_file = 'Frame_31.tif'
im = Image.open(image_file)
text = pytesseract.image_to_string(im)
#text = image_file_to_string(image_file)
#text = image_file_to_string(image_file, graceful_errors=True)
print "=====output=======\n"
print text

