from __future__ import division
import cv2
import cv2.cv as cv
import numpy as np
import rospy
import sys

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def detectCharacter(frame_data):
    bridge = CvBridge()

    try:
        opencv_img = bridge.imgmsg_to_cv2(frame_data, "bgr8")
    except CvBridgeError:
        print("Error converting image")
        exit(1)

    hsv_img = cv2.cvtColor(opencv_img, cv2.COLOR_BGR2HSV)
    #Filter out any colour that isn't a character colour

    #Yellow [15 106 173] [ 35 126 253]
    yellow_lower = np.array([20, 50, 40])
    yellow_upper = np.array([40, 255, 255])
    #Blue [93 116 159] [113 136 239]
    blue_lower = np.array([100, 50, 40])
    blue_upper = np.array([129, 255, 255])
    #Purple [145 103  23] [165 123 103]
    purple_lower = np.array([140, 40, 30])
    purple_upper = np.array([160, 255, 255])
    #Red
    red_lower = np.array([-5, 50, 40])
    red_upper = np.array([5, 255, 255])

    gray = cv2.cvtColor(opencv_img, cv2.COLOR_BGR2GRAY)
    gray = 255-gray #invert
    ret, thresh = cv2.threshold(gray, 225, 255, cv2.THRESH_BINARY)
    opencv_img_rect = opencv_img

    contours = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours[0])>0:
        for c in contours[0]:
            approx = cv2.approxPolyDP(c,0.04*cv2.arcLength(c,True),True)
            area = cv2.contourArea(c)
            if ((len(approx) == 4) & (area > 500)):
                (x, y, w, h) = cv2.boundingRect(approx)
                ar = w / float(h)
                if ar < 0.95 or ar > 1.05:
                    cv2.rectangle(opencv_img_rect,(x,y),(x+w,y+h),(0,255,0),2)
                    yellow_vote = 0
                    blue_vote = 0
                    purple_vote = 0
                    red_vote = 0
                    for xpixel in range(x, x+w):
                        for ypixel in range(y, y+h):
                            pixel = hsv_img[ypixel, xpixel]
                            if (np.greater_equal(pixel, yellow_lower).all() and np.less_equal(pixel, yellow_upper).all()):
                                yellow_vote = yellow_vote + 1
                            if (np.greater_equal(pixel, blue_lower).all() and np.less_equal(pixel, blue_upper).all()):
                                blue_vote = blue_vote + 1
                            if (np.greater_equal(pixel, purple_lower).all() and np.less_equal(pixel, purple_upper).all()):
                                purple_vote = purple_vote + 1
                            if (np.greater_equal(pixel, red_lower).all() and np.less_equal(pixel, red_upper).all()):
                                red_vote = red_vote + 1
                    voting_arr = [yellow_vote, blue_vote, purple_vote, red_vote]
                    most_votes = max(voting_arr)
                    if most_votes > int(0.4*w*h):
                        if voting_arr.index(most_votes) == 0:
                            character_name = "mustard"
                        if voting_arr.index(most_votes) == 1:
                            character_name = "peacock"
                        if voting_arr.index(most_votes) == 2:
                            character_name = "plum"
                        if voting_arr.index(most_votes) == 3:
                            character_name = "scarlett"
                        with open("cluedo_character.txt", "w") as f:
                            f.write(character_name)
                        cv2.imwrite("cluedo_character.png", opencv_img_rect)
                        return True
    return False
