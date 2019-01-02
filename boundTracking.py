# -*- coding: utf-8 -*-

import time
import argparse
import cv2
import numpy as np
from imutils.video import VideoStream
import imutils


tracker = cv2.TrackerKCF_create()

ok = True
objectSelected=False
timeCheck = time.time()
refPt = []

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
            ok = tracker.init(frame, bbox)



# Are we using the Pi Camera?
usingPiCamera = False #True
# Set initial frame size.
frameSize = (320, 240)

# Initialize mutithreading the video stream.
vs = VideoStream(src=0, usePiCamera=0, resolution=frameSize, #usePiCamera=usingPiCamera
        framerate=32).start()
# Allow the camera to warm up.
time.sleep(2.0)


frame = vs.read()
cv2.imshow("image", frame)
cv2.setMouseCallback("image", click_and_crop)

while True:
    # Get the next frame.
    frame = vs.read()


    if objectSelected:
        ok, bbox = tracker.update(frame)
        p1 = (int(bbox[0]), int(bbox[1]))
        p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
        cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)

    # If using a webcam instead of the Pi Camera,
    # we take the extra step to change frame size.
    # if not usingPiCamera:
    #     frame = imutils.resize(frame, width=frameSize[0])

    # Show video stream
    cv2.imshow("image", frame)
    key = cv2.waitKey(1) & 0xFF

    # if the `q` key was pressed, break from the loop.
    if key == ord("q"):
        break


    timeCheck = time.time()
    time.sleep(1.0/30)
# Cleanup before exit.
cv2.destroyAllWindows()
vs.stop()
