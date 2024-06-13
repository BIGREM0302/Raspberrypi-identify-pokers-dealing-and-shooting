Author@MaxZheng, ArthurChang, StanleyWu  

Project: Use Raspberrypi and picamera to identify the playing cards, and use Arduino to control the car.  

Institution: National Taiwan University  

A final project of the course: the basic technique learning and training for electronic engineering students  

The notion's linkage: https://www.notion.so/111-2-eecs-cornerstone/5-d9e2e4b5f896499f8ab886ce32ee72d4  

The package used to identify cards: OpenCV  

The templates could be resetted by running template.py in the directory Card_Imgs so that the identification will be customized to your play cards.  

*It's worth noticing that the code is designed for pi-camera. Therefore, if you want to use usb camera other than picamera, you should modify the code or it won't work.  

*The identification work flow can be simplified as the following steps:  
	1. Use pi-camera to take the photo  
	2. Turn the RGB to HSV color space  
	3. Use numpy arrays to create corresponding red and black mask(it will be more accurate than simply transform into Grayscale picture since by simply judging the "hue" values we can get a satisfying result which won't be affected by shadow and light seriously)  
	4. Use numpy.where function to turn the masked areas into pure black.  
	5. Save the modified image and turn it to Grayscale picture  
	6. Use cv2.TM_SQDIFF to match the templates  
	7. It's worth noticing tha the area of suit and rank is separated, while we also use different parameters.  
  
 
