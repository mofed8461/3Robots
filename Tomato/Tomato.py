import cv2
import imutils
import numpy as np
import math
import time

ser = True
if ser:
    import serial
    try:
        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)#ttyAMA0
        ser.open()
        print("serial working")
    except:
        print("serial not working")
        ##ser = False


STATE_SEARCHING_FOR_OBJECT = "Search for Object"
STATE_MOVE_TO_OBJECT = "Move to Object"
STATE_SEARCHING_FOR_TARGET = "Search for Target"
STATE_MOVE_TO_TARGET = "Move to Target"

currentState = STATE_SEARCHING_FOR_OBJECT

currentObject = None

def stepRight():
    print("R")
    if ser is not None and ser is not False:
        ser.write(b"r\r\n");
    pass

def stepLeft():
    print("L")
    if ser is not None and ser is not False:
        ser.write(b"l\r\n");
    pass

def move():
    print(">")
    if ser is not None and ser is not False:
        ser.write(b"m\r\n");
    pass

def stop():
    print("Stop")
    if ser is not None and ser is not False:
        ser.write(b"s\r\n");
    pass

def catchObject(val):
    print("catching..")
    if ser is not None and ser is not False:
        msg = "c" + str(val) + "\r\n"
        msg = bytes(msg, 'utf8')
        ser.write(msg)
    pass

def dropObject():
    print("Drop")
    if ser is not None and ser is not False:
        ser.write(b"d\r\n");
    pass

def canCatchObject(val):

    if ser is not None and ser is not False:
        msg = "o" + str(val) + "\r\n"
        msg = bytes(msg, 'utf8')
        ser.write(msg)
        ser.flush()
        response = ser.readline()
        response = response.decode("utf-8")
        if response.startswith("t"):
            print("can catch?yes")
            return True
        else:
            print("res:" + response)
            print("can catch?no")
            return False

    print("can catch?-YES-")
    return True



def distance(ax, ay, bx, by):
    return math.sqrt(pow(ax - bx, 2) + pow(ay - by, 2))


def getShapes(frame):


    # frame = cv2.convertScaleAbs(frame, alpha=brightness, beta=0)

    # frame = cv2.convertScaleAbs(frame, alpha=2, beta=-100)

    # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)
    # frame[:, :, 0] = cv2.equalizeHist(frame[:, :, 0])
    # frame = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR)

    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    frame = cv2.GaussianBlur(frame, (3, 3), 0)

    factor = 0.8

    # red = cv2.cvtColor(np.uint8([[[60, 90, 170]]]), cv2.COLOR_BGR2HSV)[0][0]


    # private Scalar redHsvLower = new Scalar(174, 222, 0);
    # private Scalar redHsvUpper = new Scalar(5, 255, 255);

    # filtered = cv2.inRange(frame, (30 * brightness, 30 * brightness, 40 * brightness), (60 * brightness, 60 * brightness, 80 * brightness))
    # filtered = cv2.inRange(frame, (40, 115, 90), (190, 200, 130))
    # filtered = cv2.inRange(frame, (red[0] * 0.3, 20, 20), (red[0] * 2.5, 240, 240))
    filtered = cv2.inRange(frame, (5, 100, 0), (30, 255, 255))



    # xx = np.mean(cv2.resize(filtered, (15, 10)))
    #
    # if xx > 5 and brightness < 4:
    #     brightness = brightness * 1.05
    #
    # if xx < 5 and brightness > 0.25:
    #     brightness = brightness * 0.95

    cnts = cv2.findContours(filtered, cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    return [cnts, filtered]



def ignoreLargeShapes(cnts, threshold, minPoints, maxPoints):
    shapes = []

    for c in cnts:
        if len(c) > minPoints and len(c) < maxPoints:
            area = cv2.contourArea(c)

            if area > 1 and area < threshold:  # and cv2.isContourConvex(c):
                shapes.append(c)

    return shapes

def findLargestShape(shapes):

    maxArea = 0
    output = None
    for s in shapes:
        area = cv2.contourArea(s)

        (x, y, w, h) = cv2.boundingRect(s)

        # if abs(1 - min(w / h, h / w)) < 0.3:
        #     if abs(area - w * h) < w * h * 0.5:
        if area > maxArea:
            maxArea = area
            output = s

    return output

def shapeState(rect, shape, frame):
    [x, y, w, h] = cv2.boundingRect(shape)

    dir = None
    height = None
    found = shape is not None
    if found:
        if x + w * 0.5 > rect[1] * 0.5:
            dir = "right"
        else:
            dir = "left"

        height = 1.0 - (y + h * 0.5) / rect[0]


    hsv = None
    if found:
        hsv = frame[int(y + h * 0.4):int(y + h * 0.6),int(x + w * 0.4):int(x + w * 0.6)]
        hsv = cv2.cvtColor(hsv, cv2.COLOR_BGR2HSV)
        hsv = hsv.mean(axis=0).mean(axis=0)
        hsv = [int(hsv[0]), int(hsv[1]), int(hsv[2])]
    return [found, dir, w * h, height, hsv]

def getShapesTarget(frame):

    # frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # frameHSV = cv2.GaussianBlur(frameHSV, (3, 3), 0)


    # color = np.uint8([[[115, 124, 161]]])
    # color = cv2.cvtColor(color, cv2.COLOR_RGB2HSV)[0][0]

    # filtered = cv2.inRange(frame, (30 * brightness, 30 * brightness, 40 * brightness), (60 * brightness, 60 * brightness, 80 * brightness))
    filtered = cv2.inRange(frame, (0, 0, 0), (50, 50, 50))
    # filtered = cv2.inRange(frameHSV, (120, 10, 120), (170, 70, 220)) ## sky

    [h,w,_] = frame.shape
    pt1 = (int(w * 0.3), int(h * 0.7))
    pt2 = (int(w * 0.7), h)
    cv2.rectangle(filtered, pt1, pt2, 0, -1)


    # filtered = cv2.inRange(frameHSV, (120, 10, 140), (170, 50, 220))

    cnts = cv2.findContours(filtered, cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    return [cnts, filtered]

def ignoreLargeShapesOptimized(cnts, areaMin, areaMax, minPoints, maxPoints):
    shapes = []

    for c in cnts:

        peri = cv2.arcLength(c, True)
        c = cv2.approxPolyDP(c, 0.04 * peri, True)

        if len(c) > minPoints and len(c) < maxPoints:
            area = cv2.contourArea(c)

            if area > areaMin:# and area < areaMax:  # and cv2.isContourConvex(c):
                shapes.append(c)

    return shapes




cap = cv2.VideoCapture(0)

ser.write(b'q\r\n')

brightness = 1
while (True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    # frame = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
    rect = frame.shape

    print("----")
    # print(frame[0][0])
    [shapes, filtered] = getShapes(frame)
    [shapesTarget, filteredTarget] = getShapesTarget(frame)

    shapes = ignoreLargeShapes(shapes, 90000, 40, 10000)
    shapesTarget = ignoreLargeShapesOptimized(shapesTarget, rect[0] * 0.01 * rect[1] * 0.01, 10000, 3, 13)

    shape = findLargestShape(shapes)
    target = findLargestShape(shapesTarget)

    [TomatoFound, DirectionToTomato, TomatoSize, TomatoY, TomatoHSV] = shapeState(rect, shape, frame)
    [TargetFound, DirectionToTarget, TargetSize, TargetY, TargetHSV] = shapeState(rect, target, frame)


    cv2.putText(frame, "tomato " + str([TomatoFound, DirectionToTomato, TomatoSize, TomatoY, TomatoHSV]), (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
    cv2.putText(frame, "target " + str([TargetFound, DirectionToTarget, TargetSize, TargetY, TargetHSV]), (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
    if TomatoFound:
        cv2.drawContours(frame, [shape], 0, (0, 255, 0), 2)

    if TargetFound:
        cv2.drawContours(frame, [target], 0, (255, 255, 0), 2)










    nextState = currentState

    if currentState == STATE_SEARCHING_FOR_OBJECT:

        if TomatoFound:
            nextState = STATE_MOVE_TO_OBJECT
            move()
        else:
            stop()
            stepRight()
    elif currentState == STATE_MOVE_TO_OBJECT:
        move()
        if not TomatoFound:
            nextState = STATE_SEARCHING_FOR_OBJECT
        else:
            if DirectionToTomato == "right":
                stepRight()
            else:
                stepLeft()

        if canCatchObject(TomatoY):
            stop()
            catchObject(TomatoY)
            catchObject(0)
            nextState = STATE_SEARCHING_FOR_TARGET
    elif currentState == STATE_SEARCHING_FOR_TARGET:
        if TargetFound:
            move()
            nextState = STATE_MOVE_TO_TARGET
        else:
            stepRight()

    elif currentState == STATE_MOVE_TO_TARGET:

        if TargetFound:
            if DirectionToTarget == "right":
                stepRight()
            else:
                stepLeft()
        else:
            stop()
            dropObject()
            nextState = STATE_SEARCHING_FOR_OBJECT

    print(currentState)
    currentState = nextState




        # cv2.drawContours(frame, [shape], 0, (0, 255, 0), 2)
    #     cv2.putText(frame, name, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

    if cv2.waitKey(1) & 0xFF == ord(' '):
        frame = filteredTarget

    cv2.imshow("frame", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

exit(0)