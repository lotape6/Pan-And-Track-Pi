import io
import socket
import struct
from PIL import Image
import cv2
import numpy as np
import argparse
from imutils.video import VideoStream
import imutils


# SET INITIAL PARAMETERS -------------------------------------------------------
frameSize = (640, 480)
frameCenter = (frameSize[0]/2,frameSize[1]/2)
ok = True
objectSelected=False
#timeCheck = time.time()
refPt = []

tracker = cv2.TrackerKCF_create()

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
        print('Image is %dx%d' % old_image.size)
        old_image.verify()
        print('Image is verified')
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
            # Draw rectangle and center
            cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
            cv2.circle(frame,obj_center,2,(255,0,0),-1)


        # SHOW IMAGE RESULTS ---------------------------------------------------
        cv2.imshow("Stream",frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
    	       break

# CLOSE CONNECTIONS ------------------------------------------------------------
finally:
    connection.close()
    server_socket.close()
