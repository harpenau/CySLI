# import the necessary packages
from imutils.video.pivideostream import PiVideoStream
from imutils.video import VideoStream
import numpy as np
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
from imutils.video import FPS
import argparse
import imutils
# capture camera and start playing at 30 frames per second
vs=PiVideoStream(framerate=30)
vs.start()

# give time for the camera to warm up. Actually necessary
time.sleep(2.0)

# initialize writer
writer=None
(h,w)=(None,None)

# start the threading and start the frames per second counter
fps=FPS()
fps.start()

# define the boundaries of the colors
boundariesHsv=[
			([25,20,20],[35,255,255]),#yellow
			([90,0,0],[140,255,255]),#blue
			([0,0,0],[5,255,255]),#pink
			([170,0,0],[180,255,255])#pink
			]

# add a print statement to know when setup is done
print("[INFO] entering while loop")

# loop until I say not to(cntl-c)
while True:
	try:
		
		# take the most recent image returned by the camera
		image=vs.read()

		# check writer
		if writer is None:
			# store dimentions and initialize writer
			(h,w)=image.shape[:2]
			writer=cv2.VideoWriter('Test.avi',cv2.VideoWriter_fourcc(*'MJPG'),40,(w,h),True)
		
		# convert the image to HSV colorspace
		imageHsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
		
		# loop over the boundaries for each color
		for (lower, upper) in boundariesHsv:
			# create NumPy arrays from the boundaries
			lower = np.array(lower, dtype = "uint8")
			upper = np.array(upper, dtype = "uint8")
			
			# checks each pixel to see if it's in the boundaries then prepares image to 
			# find contours and finds contours
			mask = cv2.inRange(imageHsv, lower, upper)
			blurred=cv2.GaussianBlur(mask,(5,5),0)
			thresh=cv2.threshold(blurred,60,255,cv2.THRESH_BINARY)[1]
			_,cnts,_=cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
			
			# loops over all contours and finds ones with 4 corners then draws the contours 
			for cnt in cnts:
				perimeter=cv2.arcLength(cnt,True)
				shape=cv2.approxPolyDP(cnt, 0.02*perimeter,True)
				if len(shape)==4 and cv2.contourArea(cnt)>1000:
					cv2.drawContours(image,[cnt],-1,(0,255,0),2)
		
		# writer saves the image
		writer.write(image)
		
		# updates the fps
		fps.update()
		
	# exits if I press cntl-c
	except KeyboardInterrupt:
		break

# stops fps counter and prints fps		
fps.stop()
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
print("[INFO] cleaning up...")

# cleaning up resources
vs.stop()
writer.release()



