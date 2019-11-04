# import the necessary packages
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
 
# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=10, help="max buffer size")
args = vars(ap.parse_args())

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
greenLower = (40, 60, 50)
greenUpper = (240, 240, 190)
pts = deque(maxlen=args["buffer"])
 
# if a video path was not supplied, grab the reference
# to the webcam
if not args.get("video", False):
    vs = VideoStream(src=0).start()
 
# otherwise, grab a reference to the video file
else:
    vs = cv2.VideoCapture(args["video"])
 
# allow the camera or video file to warm up
time.sleep(0.5)


# keep looping
while True:
    # grab the current frame
    frame = vs.read()
    #cv2.imwrite('/home/pi/Desktop/tmp.png', frame)
    # handle the frame from VideoCapture or VideoStream
    frame = frame[1] if args.get("video", False) else frame
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)

    # equalize the histogram of the Y channel
    frame[:,:,0] = cv2.equalizeHist(frame[:,:,0])

    # convert the YUV image back to RGB format
    frame = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR)
    
    
    # if we are viewing a video and we did not grab a frame,
    # then we have reached the end of the video
    if frame is None:
        break
    
    img = cv2.medianBlur(frame,5)
    cimg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    circles = cv2.HoughCircles(cimg, cv2.HOUGH_GRADIENT,1,100,param1=100,param2=40,minRadius=30,maxRadius=500)
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
            ixMin = np.uint16(i[0]-(i[2]*0.707106))
            ixMax = np.uint16(i[0]+(i[2]*0.707106))
            iyMin = np.uint16(i[1]-(i[2]*0.707106))
            iyMax = np.uint16(i[1]+(i[2]*0.707106))
            mean, std = cv2.meanStdDev(frame[ixMin:ixMax, iyMin:iyMax])
            stdMean = np.mean(std)
            
            cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),1) # draw the outer circle

            #cv2.putText(frame,"m:" + str(np.uint16(mean)), (i[0],i[1]), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1)
            #cv2.putText(frame,"s:" + str(std), (i[0],i[1]-20), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1)
            
            if stdMean > 30 or stdMean < 0:
                continue
            
            if abs(np.mean(std - [stdMean, stdMean, stdMean])) > 30:
                continue
            
            print(mean)
            cv2.putText(cimg,"m:" + str(np.uint16(mean)), (i[0],i[1]), cv2.FONT_HERSHEY_PLAIN, 1, 255, 1)
            cv2.putText(cimg,"s:" + str(np.uint16(std)), (i[0],i[1] + 16), cv2.FONT_HERSHEY_PLAIN, 1, 255, 1)

            
            if abs(np.mean(mean - [110, 120, 120])) > 20:
                continue
            
            cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3) # draw the center of the circle

    
    # show the frame to our screen
    cv2.imshow("Frame", cimg)
    key = cv2.waitKey(1) & 0xFF
 
    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break
 
# if we are not using a video file, stop the camera video stream
if not args.get("video", False):
    vs.stop()
 
# otherwise, release the camera
else:
    vs.release()
 
# close all windows
cv2.destroyAllWindows()