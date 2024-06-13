import cv2
import numpy as np
import os
import argparse
from picamera2 import Picamera2
from libcamera import Transform

picam2=Picamera2()

img_width=840
img_height=616

#paramters:
threshvalue=100
exposuretime=50000
black_value=122
black_value_suit=150

#define what kind
def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", default="rank", type=str)
    parser.add_argument("--type", default="ace", type=str)
    return parser.parse_args()

#define the region
#for the rank
x=725
y=220
w=1008-740
h=635-242

#for the suit
x_=740
y_=685
w_=1005-775
h_=948-664

camera_config = picam2.create_still_configuration(
	main={"size": (1680, 1232), "format": "RGB888"},
	controls={"ExposureTime": exposuretime, "AnalogueGain": 1.0},
	transform=Transform(vflip=True, hflip=True)
)

#debug photo
def show(img):
    while True:
        cv2.imshow('template',cv2.resize(img, (img_width, img_height)))
        key=cv2.waitKey(1)
        if key == 27:
            break
    return

args = parse_args()

# Fetch the parameters
mode = args.mode
type = args.type

name = mode+"_"+type+".png"

picam2.configure(camera_config)
picam2.start()
picam2.capture_file(name)
picam2.stop()

image=cv2.imread(name)

show(image)

hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Define the range for red color in HSV
lower_red1 = np.array([0, 120, 70])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([170, 120, 70])
upper_red2 = np.array([180, 255, 255])

# Define the range for black color in HSV
lower_black = np.array([0, 0, 0])
upper_black = np.array([180, 255, black_value_suit])

# Create masks for the red color
mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
mask_red = cv2.bitwise_or(mask_red1, mask_red2)

# Create mask for the black color
mask_black = cv2.inRange(hsv, lower_black, upper_black)

# Combine the masks for red and black regions
mask_combined = cv2.bitwise_or(mask_red, mask_black)

# Invert the mask to get regions that are not red and not black
mask_not_red_black = cv2.bitwise_not(mask_combined)

# Create a white image
white_image = np.ones_like(image) * 255

# Create a black image
black_image = np.zeros_like(image)

# Apply the inverted mask to make the non-red and non-black regions white
result = np.where(mask_not_red_black[:, :, np.newaxis] == 255, white_image, image)

# Apply the red and black mask to make the regions black
result = np.where(mask_combined[:, :, np.newaxis] == 255, black_image, result)

gray=cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
show(gray)
_, thresh_binary = cv2.threshold(gray, threshvalue, 255, cv2.THRESH_BINARY)
show(thresh_binary)
blurred = cv2.GaussianBlur(result, (3,3), 1.4)
show(blurred)

if mode == "rank":
    template_region=blurred[y:y+h, x:x+w]
else:
    template_region=blurred[y_:y_+h_, x_:x_+w_]
show(template_region)

cv2.imwrite(name,template_region)

cv2.destroyAllWindows()
