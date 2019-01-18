# Pan-And-Track-Pi
Simple object tracker based on OpenCV with the Raspberry Pi camera and a 9g servos Pan and Tilt

## Dependencies

Before running the main script boundTracking.py you should install if haven't got: OpenCV, pip, imutils and picamera.

For installing OpenCV you can visit this wonderful tutorial (and remember to have fresh beer near you, it will take a looong time):

	https://www.pyimagesearch.com/2017/09/04/raspbian-stretch-install-opencv-3-python-on-your-raspberry-pi/

The others are really straight forward:

	sudo apt-get install python-pip
	pip install picamera
	pip install imutils
	pip install pid
	pip install RPi.GPIO
	pip install socket
	pip install pil

## Start Guide

Firstly, run server_script on your server.
Secondly, run client_script on Raspberry Pi. NOTE: Remember to specify the correct server address
Finally, select an object to track on "Stream" window and let the system do the rest :)
