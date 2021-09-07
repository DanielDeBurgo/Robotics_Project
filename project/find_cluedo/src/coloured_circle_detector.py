from __future__ import division
import cv2
import cv2.cv as cv
import numpy as np
import rospy
import sys

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

im_pub = rospy.Publisher('/cleudo/mask_image', Image, queue_size=100)

def detectCircle(frame_data):
    detected_red_circle = False
    detected_green_circle = False
    bridge = CvBridge()
    sensitivity = 5
    try:
        opencv_img = bridge.imgmsg_to_cv2(frame_data, "bgr8")
    except CvBridgeError:
        print("Error converting image")
        exit(1)
    #Filter out any colour that isn't red or green
    hsv_red_lower = np.array([0 - sensitivity, 100, 102])
    hsv_red_upper = np.array([0 + sensitivity, 255, 128])
    hsv_green_lower = np.array([70 - sensitivity, 150, 73])
    hsv_green_upper = np.array([70 + sensitivity, 255, 200])

    try:
        Hsv_image = cv2.cvtColor(opencv_img, cv2.COLOR_BGR2HSV)
    except:
        return False
    red_mask = cv2.inRange(Hsv_image, hsv_red_lower, hsv_red_upper)
    green_mask = cv2.inRange(Hsv_image, hsv_green_lower, hsv_green_upper)

    colour_mask = cv2.bitwise_or(green_mask, red_mask)
    colour_mask_image=cv2.bitwise_and(opencv_img,opencv_img, mask=colour_mask)

    im_pub.publish(bridge.cv2_to_imgmsg(colour_mask_image))

    contourArr = [cv2.findContours(red_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE), cv2.findContours(green_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)]
    # Loop over the contours
    count = 0
    c_red= 0
    c_green = 0
    for contours in contourArr:
        if len(contours[0])>0:

            # There are a few different methods for identifying which contour is the biggest
            # Loop throguht the list and keep track of which contour is biggest or
            # Use the max() method to find the largest contour
            for c in contours[0]:
                approx = cv2.approxPolyDP(c,0.01*cv2.arcLength(c,True),True)
                area = cv2.contourArea(c)
                if ((len(approx) > 8) & (cv2.contourArea(c) > 200) ):
                    M = cv2.moments(c)
                    cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
                    (x, y), radius = cv2.minEnclosingCircle(c)
                    height, width, channels = Hsv_image.shape
                    if (x + 2*radius > width or x - 2*radius < 0 or y - 2*radius < 0 or y + 2 * radius > height):
                        break
                    # Then alter the values of any flags
                    if count == 0:
                        cv2.circle(colour_mask_image,(int(x),int(y)),int(radius),(255, 211, 0),1)
                        c_red = cv2.contourArea(c)
                        detected_red_circle = True
                    elif count == 1:
                        cv2.circle(colour_mask_image,(int(x),int(y)),int(radius),(255, 211, 0),1)
                        cv2.circle(opencv_img,(int(x),int(y)),int(radius),(255, 211, 0),1)
                        c_green = cv2.contourArea(c)
                        detected_green_circle = True
                        cv2.imwrite("green_circle.png", opencv_img)
        count = count + 1
    if detected_red_circle and detected_green_circle:
        if c_red > c_green:
            return "red"
        else:
            return "green"
    elif detected_green_circle:
        return "green"
    elif detected_red_circle:
        return "red"
    else:
        return False
