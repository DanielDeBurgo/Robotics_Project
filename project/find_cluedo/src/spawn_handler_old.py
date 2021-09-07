#!/usr/bin/env python
import rospy
import sys
import actionlib
import tf2_ros
import tf
import math
import numpy as np
from geometry_msgs.msg import Point, PoseStamped, Twist, PointStamped, Pose
from tf2_geometry_msgs import PointStamped as GPointStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from sensor_msgs.msg import Image
import coloured_circle_detector
import character_detector

buffer = tf2_ros.Buffer()
robot_frame = "base_footprint"

class SpawnHandler:
    def __init__(self, dx1, dy1, dx2, dy2, rx1, ry1, rx2, ry2):
        self.cluedo_flag = False
        self.room_no = 0
        self.move_speed = 0.5
        self.green_flag = False
        self.red_flag = False
        self.img_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.circle_callback, queue_size=1000)
        self.im_pub = rospy.Publisher('/cleudo/colour_detector', Image, queue_size=100)
        self.move_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size = 10)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(5))
        self.first_door = self.make_map_point(dx1, dy1)
        self.first_room = self.make_map_point(rx1, ry1)
        self.second_door = self.make_map_point(dx2, dy2)
        self.second_room = self.make_map_point(rx2, ry2)
        #mid points between door and center of room
        self.mx1 = (rx1 + dx1)/2
        self.my1 = (ry1 + dy1)/2
        self.mx2 = (rx2 + dx2)/2
        self.my2 = (ry2 + dy2)/2
        self.first_mid = self.make_map_point(self.mx1,self.my1)
        self.second_mid = self.make_map_point(self.mx2, self.my2)
        self.d1 = self.distance(self.mx1, self.my1, rx1, ry1)
        self.d2 = self.distance(self.mx2, self.my2, rx2, ry2)
        self.nx1 = self.mx1
        self.ny1 = self.my1
        self.nx2 = self.mx2
        self.ny2 = self.my2
        self.first_mid2 = self.make_map_point(self.nx1,self.ny1)
        self.second_mid2 = self.make_map_point(self.nx2, self.ny2)

        self.turn(360)
        if self.green_flag:
            #if no then find which door has the smallest angle from x axis
            angle1 = abs(self.angle_to_point_from_base(self.first_door.point, "map"))
            angle2 = abs(self.angle_to_point_from_base(self.second_door.point, "map"))
            if angle1 > angle2:
                self.move_to_point(self.second_door.point)
            else:
                self.move_to_point(self.first_door.point)
            self.green_flag = False

        ## CHECK DOORS ##
        if self.closest_point() == 1:
            if not self.check_door(self.first_door, self.first_room):
                self.check_door(self.second_door, self.second_room)
        else:
            if not self.check_door(self.second_door, self.second_room):
                self.check_door(self.first_door, self.first_room)

        if self.green_flag:
            self.choose_door()

        self.choose_door()

    def distance(self, x1, y1, x2, y2):
        d = (((x2-x1)**2 + (y2-y1)**2) ** 0.5)
        return d


    def circle_callback(self, img):
        self.im_pub.publish(img)
        if self.green_flag == True:
            cm = character_detector.detectCharacter(img)
            if cm == True:
                self.cluedo_flag = True
                print ("cluedo character detected")
        else:
            ret = coloured_circle_detector.detectCircle(img)
            if ret == "red":
                self.red_flag = True
            elif ret == "green":
                self.green_flag = True
                #self.img_sub.unregister()



    def closest_point(self):
        robot = PointStamped()
        robot.header.frame_id = "base_footprint"
        robot.header.stamp = rospy.Time.now()
        robot.point.x = 0
        robot.point.y = 0

        r1 = self.transform_to_basefootprint(self.first_door)
        d1 = self.euclidean_dist(robot.point, r1.point)
        r2 = self.transform_to_basefootprint(self.second_door)
        d2 = self.euclidean_dist(robot.point, r2.point)

        if d1 > d2:
            return 2
        else:
            return 1

    def transform_to_basefootprint(self, point):
        return buffer.transform(point, "base_footprint", timeout=rospy.Duration(10.0))

    def make_map_point(self, x, y):
        point = GPointStamped()
        point.header.frame_id = "map"
        point.point.x = x
        point.point.y = y
        return point

    def find_cluedo(self, center, mid, mid2):
        #move to center of room
        self.move_to_point(center)

        #scan 360 degrees from the center of the room
        if self.turn2(360):
            return self.cluedo_flag
        rospy.sleep(1)

        #move forward towards wall and scan 360 degrees
        self.move_to_point(mid2)
        if self.turn2(360):
            return self.cluedo_flag
        rospy.sleep(1)

        #move towards door and scan 360 degrees
        self.move_to_point(mid)
        if self.turn2(360):
            return self.cluedo_flag
        rospy.sleep(1)

        return self.cluedo_flag


    def check_door(self, door, room):
        #move to door
        self.move_to_point(door.point)

        #get angle to room center
        angle = self.angle_to_point_from_base(room.point, "map")

        #turn to center of room and then scan to either side of the door
        rospy.sleep(1)
        if self.turn(angle, rad=True):
            return self.green_flag
        rospy.sleep(1)

        #scan 45 degrees to either side to look for coloured circle
        if self.turn(-45):
            return self.green_flag
        rospy.sleep(1)

        if self.turn(90):
            return self.green_flag
        rospy.sleep(1)

        angle = self.angle_to_point_from_base(room.point, "map")
        self.turn(angle) #turn back to room center

        return self.green_flag

    def turn2(self, angle, rad=False):
        #return if cluedo flag is already true
        if self.cluedo_flag:
            return True
        #if angle is in degrees convert to radians
        if not rad:
            angle = angle * math.pi/180

        turn_speed = max(1 * (abs(angle)/(2*math.pi)), 0.5)

        #set twist for turn message
        turn = Twist()
        left = True
        if angle >= 0:
            turn.angular.z = turn_speed
        else:
            left = False
            turn.angular.z = -turn_speed

        #rotate
    	t0 = rospy.Time.now().to_sec()
    	curr_angle = 0
        if not left:
            angle = angle * -1
        while(curr_angle < angle and self.green_flag == False):
            self.move_pub.publish(turn)
            t1 = rospy.Time.now().to_sec()
            curr_angle = turn_speed * (t1-t0)

            if self.cluedo_flag == True:
                break

        return self.cluedo_flag


    def choose_door(self):
        if self.closest_point() == 1:
            #self.move_to_point(self.first_room.point)
            if self.find_cluedo(self.first_room.point, self.first_mid.point, self.first_mid2.point):
                print("cluedo image found")
            else:
                print("cluedo image not found")
        else:
            #self.move_to_point(self.second_room.point)
            if self.find_cluedo(self.second_room.point, self.second_mid.point, self.second_mid2.point):
                print("cluedo image found")
            else:
                print("cluedo image not found")

    def angle_to_point_from_base(self, point, frame_id):
        #compute angle from x axis to point
        point_stamped = PointStamped()
        point_stamped.header.frame_id = frame_id
        point_stamped.point = point
        point_base = buffer.transform(point_stamped, "base_footprint", timeout=rospy.Duration(10.0))
        dx = point_base.point.x
        dy = point_base.point.y
        return math.atan2(dy, dx)

    def turn(self, angle, rad=False):
        #dont turn in green flag is true
        if self.green_flag:
            return True

        #if angle is in degrees convert to radians
        if not rad:
            angle = angle * math.pi/180

        turn_speed = max(1 * (abs(angle)/(2*math.pi)), 0.5)

        #set twist for turn message
        turn = Twist()
        left = True
        if angle >= 0:
            turn.angular.z = turn_speed
        else:
            left = False
            turn.angular.z = -turn_speed

        #rotate
    	t0 = rospy.Time.now().to_sec()
    	curr_angle = 0
        if not left:
            angle = angle * -1
        while(curr_angle < angle and self.green_flag == False):
            self.move_pub.publish(turn)
            t1 = rospy.Time.now().to_sec()
            curr_angle = turn_speed * (t1-t0)

            #check if the green circle is seen. If seen then stop turn and set flag
            #green_flag = True
            #break

        return self.green_flag

    def move_to_point(self, point, goal_pose=None):
        #set goal_pose
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        if not goal_pose == None:
            goal.target_pose.pose = goal_pose
        else:
            goal_pose = Pose()
            goal_pose.position = point
            goal_pose.orientation.x = 0
            goal_pose.orientation.y = 0
            goal_pose.orientation.z = 0.5
            goal_pose.orientation.w = 0
            goal.target_pose.pose = goal_pose

        #send move base goal and wait for the result
        self.move_base.send_goal(goal)
        self.goal_sent = True
        success = self.move_base.wait_for_result(rospy.Duration(60))
        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            result = True
        else:
            result = False

        return result

    def euclidean_dist(self, point1, point2):
        return math.sqrt(pow(point1.x - point2.x, 2) +
                         pow(point1.y - point2.y, 2))

def main(args):
    rospy.init_node("spawn_to_door", anonymous = True)
    listener = tf2_ros.TransformListener(buffer)
    handler = SpawnHandler(-1.43, 1.15, 3.4, 1.15, -2.3, 5.63, 2.3, 5.63)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down find_cleudo")



if __name__ == "__main__":
    main(sys.argv)
