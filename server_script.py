import io
import socket
import struct
from PIL import Image
import cv2
import numpy as np
import argparse
from imutils.video import VideoStream
import imutils
from time import sleep
import PID
import pigpio

pi = pigpio.pi('192.168.1.15')
# SET INITIAL PARAMETERS -------------------------------------------------------
frameSize = (640, 480)
frameCenter = (frameSize[0]/2,frameSize[1]/2)
ok = True
objectSelected=False
#timeCheck = time.time()
refPt = []
Pan_gpioPIN = 23
Tilt_gpioPIN = 22
Pan_initPosition = 1500
Tilt_initPosition = 1500
Prev_pan_pose = Pan_initPosition
Prev_tilt_pose = Tilt_initPosition
Pan_max_range = 700
Tilt_max_range = 400

tracker = cv2.TrackerKCF_create()

# PID'S DEFINITION -------------------------------------------------------------
Pan_pidTarget = 0
Tilt_pidTarget = 0

Pan_Kp = -2
Pan_Ki = 1
Pan_Kd = 0
Tilt_Kp = 2
Tilt_Ki = 1
Tilt_Kd = 0

Pan_pid = PID.PID(Pan_Kp, Pan_Ki, Pan_Kd)
Pan_pid.SetPoint = Pan_pidTarget
Pan_pid.setSampleTime(0.1)
Tilt_pid = PID.PID(Tilt_Kp, Tilt_Ki, Tilt_Kd)
Tilt_pid.SetPoint = Tilt_pidTarget
Tilt_pid.setSampleTime(0.1)

# GPIO DEFINITION --------------------------------------------------------------
pi.set_mode(Pan_gpioPIN, pigpio.OUTPUT)
pi.set_mode(Tilt_gpioPIN, pigpio.OUTPUT)

# DEFINE TARGET OBJECT SELECTION -----------------------------------------------
def click_and_crop(event, x, y, flags, param):

    global objectSelected, refPt, cropping, ok, tracker, frame
    if not objectSelected:

        if event == cv2.EVENT_LBUTTONDOWN:
            refPt = [x, y]
            cropping = True

        # check to see if the left mouse button was released
        elif event == cv2.EVENT_LBUTTONUP:
            # record the ending (x, y) coordinates and indicate that
            # the cropping operation is finished
            refPt.append(x)
            refPt.append(y)
            cropping = False
            objectSelected = True
            print("""x0: %d, y0: %d
                     x1: %d, y1: %d""" %(refPt[0],refPt[1],refPt[2],refPt[3]) )
            bbox = (refPt[0], refPt[1], refPt[2]-refPt[0],refPt[3]-refPt[1])
            center_bbox = (refPt[0] + (refPt[2]-refPt[0])/2 , refPt[1] + (refPt[3]-refPt[1])/2)
            print("center_bbox: x: %d  y: %d" %center_bbox)
            ok = tracker.init(frame, bbox)

# CALCULATE ANGLE AND MOVE SERVOS FUNCTION -------------------------------------
def setServoAngle(servo, angle):
	#pi.set_PWM_frequency(servo,50)
	if servo == Pan_gpioPIN:
		if angle > (Pan_initPosition + Pan_max_range):
			angle =  Pan_initPosition + Pan_max_range
		elif angle < (Pan_initPosition - Pan_max_range):
			 angle =  Pan_initPosition - Pan_max_range
	if servo == Tilt_gpioPIN:
		if angle > (Tilt_initPosition + Tilt_max_range):
			angle =  Tilt_initPosition + Tilt_max_range
		elif angle < (Tilt_initPosition - Tilt_max_range):
			 angle =  Tilt_initPosition - Tilt_max_range

	pulsewidth = angle
	#dutyCycle = angle / 18. + 3.
	pi.set_servo_pulsewidth(servo,pulsewidth)
	#pi.set_PWM_dutycycle(servo,dutyCycle)
	sleep(0.1)
	pi.set_servo_pulsewidth(servo,0)

# CALCULATE PID ERROR FUNCTION -------------------------------------------------
def calculatePIDerror(i,obj_center):
	# The same function for both servos
	# Known: Image frame center and object position in frame (from tracking)
	pid_error = frameCenter[i] - obj_center
	return pid_error

# ALLOW MOUSE ------------------------------------------------------------------
cv2.namedWindow("Stream");
cv2.setMouseCallback("Stream", click_and_crop)

# START SOCKET SERVER LISTENING FOR CONNECTIONS --------------------------------
server_socket = socket.socket()
server_socket.bind(('0.0.0.0', 8000)) #(0.0.0.0 means all interfaces)
server_socket.listen(0)

# ACCEPT A SINGLE CONNECTION ---------------------------------------------------
connection = server_socket.accept()[0].makefile('rb')

# START RECEIVING DATA ---------------------------------------------------------
            # This program is design to send first the length of the image as a
            # 32-bit integer (in Little Endian format), then this will be
            # followed by the bytes of image data.

            #          IMAGE LENGTH (4 bytes) + IMAGE DATA (# bytes)

            # A value of length = 0 means the sending of data is finished
try:
    setServoAngle(Pan_gpioPIN, Pan_initPosition)
    setServoAngle(Tilt_gpioPIN, Tilt_initPosition)
    while True:
        # Read the length of the image. If the length is zero, quit the loop
        image_len = struct.unpack('<L', connection.read(struct.calcsize('<L')))[0]
        if not image_len:
            break
        # Construct a stream to hold the image data and read the image
        # data from the connection
        image_stream = io.BytesIO()
        image_stream.write(connection.read(image_len))
        # Rewind the stream
        image_stream.seek(0)
        # Open data as an image with PIL and convert to RGB
        old_image = Image.open(image_stream).convert('RGB')
        #print('Image is %dx%d' % old_image.size)
        old_image.verify()
        #print('Image is verified')
        # Convert image to array to proccess it with opencv
        frame = np.asarray(old_image)

        # START IMAGE PROCESSING -----------------------------------------------
        # Draw a circle in the center of the frame
        cv2.circle(frame,frameCenter,5,(0,255,0),-1)
        if objectSelected:
            ok, bbox = tracker.update(frame)
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            obj_center = (p1[0] + (p2[0]-p1[0])/2 , p1[1] + (p2[1]-p1[1])/2)
            print("Object center: ", obj_center)
            # Draw rectangle and center
            cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
            cv2.circle(frame,obj_center,2,(255,0,0),-1)
		    # Calculate error for both servos
            Pan_pidError = calculatePIDerror(0,obj_center[0]) #"pan" = 0
            Tilt_pidError = calculatePIDerror(1,obj_center[1])
            # Update pid signal control value
            Pan_pid.update(Pan_pidError)
            Tilt_pid.update(Tilt_pidError)
            # Calculate value to send to servos: position + pid output
            if (Pan_pid.output < 400) and (Pan_pid.output >-400):
                 Pan_Position = Pan_initPosition + Pan_pid.output
                 Tilt_Position = Tilt_initPosition + Tilt_pid.output
            print("New Pan output:    ", Pan_pid.output)
            print("New Tilt output:    ", Tilt_pid.output)
            print("New Pan Angle:    ", Pan_Position)
            print("New Tilt Angle:    ", Tilt_Position)
            # Move servos to the calculated position
            setServoAngle(Pan_gpioPIN, Pan_Position)
            setServoAngle(Tilt_gpioPIN, Tilt_Position)

        # SHOW IMAGE RESULTS ---------------------------------------------------
        cv2.imshow("Stream",frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
    	       break

# CLOSE CONNECTIONS ------------------------------------------------------------
finally:
    connection.close()
    server_socket.close()
