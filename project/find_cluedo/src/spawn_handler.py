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
from sensor_msgs.msg import Image, LaserScan
from kobuki_msgs.msg import BumperEvent
import coloured_circle_detector
import character_detector
import random

buffer = tf2_ros.Buffer()
robot_frame = "base_footprint"


class SpawnHandler:
    def __init__(self):
        dx1, dy1 = rospy.get_param("find_cluedo/room1_entrance_xy")
        dx2, dy2 = rospy.get_param("find_cluedo/room2_entrance_xy")
        rx1, ry1 = rospy.get_param("find_cluedo/room1_centre_xy")
        rx2, ry2 = rospy.get_param("find_cluedo/room2_centre_xy")
        self.cluedo_flag = False
        self.room_no = 0
        self.move_speed = 0.5
        self.green_flag = False
        self.red_flag = False
        self.stop = False
        self.img_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.circle_callback, queue_size=1000)
        self.im_pub = rospy.Publisher('/cleudo/colour_detector', Image, queue_size=100)
        self.move_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size = 10)
        self.bumper_sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, callback=self.callback, callback_args=self.move_pub)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.laser_sub = rospy.Subscriber('scan', LaserScan, self.laser_callback)
        self.move_base.wait_for_server(rospy.Duration(5))
        self.first_door = self.make_map_point(dx1, dy1)
        self.first_room = self.make_map_point(rx1, ry1)
        self.second_door = self.make_map_point(dx2, dy2)
        self.second_room = self.make_map_point(rx2, ry2)
        #laser detected obstacles vales
        self.l0 = 0 # at 0 degrees
        self.l90 = 0 # at 90 degrees
        self.l180 = 0 # at 180 degrees


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

    # updates values from the laser scanner
    def laser_callback(self, msg):
        self.l0 = msg.ranges[0]
        self.l90 = msg.ranges[50]
        self.l180 = msg.ranges[99]
        #print ("laser scanned callback")


    def distance(self, x1, y1, x2, y2):
        d = (((x2-x1)**2 + (y2-y1)**2) ** 0.5)
        return d

    def callback(self, data, move_pub): #callback to stop
	    if data.state == 1:
		    self.stop = True


    def circle_callback(self, img):
        self.im_pub.publish(img)
        if self.green_flag == True:
            cm = character_detector.detectCharacter(img)
            if cm == True:
                self.cluedo_flag = True
                print ("cluedo character detected")
                exit()
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

    #creates a point of the map
    def make_map_point(self, x, y):
        point = GPointStamped()
        point.header.frame_id = "map"
        point.point.x = x
        point.point.y = y
        return point

    def ping(self, center):
        rate = rospy.Rate(10) #10hz
    	desired_velocity = Twist()
    	desired_velocity.linear.x = 0.5
    	desired_velocity.angular.z = 0
        move_no = 1
        if self.turn2(225):
            return self.cluedo_flag
        while self.cluedo_flag == False and move_no == 1:
            for i in range(5):
                self.move_pub.publish(desired_velocity)
                rate.sleep()
            if self.l90 < 1 :
                if self.turn2(360):
                    return self.cluedo_flag
                move_no = 2
                if self.turn2(180):
                    return self.cluedo_flag
        while self.cluedo_flag == False and move_no == 2:
            for i in range(5):
                self.move_pub.publish(desired_velocity)
                rate.sleep()
            if self.l90 < 1 :
                move_no = 3
                if self.turn2(360):
                    return self.cluedo_flag
                if self.turn2(45):
                    return self.cluedo_flag
        while self.cluedo_flag == False and move_no == 3:
            for i in range(5):
                self.move_pub.publish(desired_velocity)
                rate.sleep()
            if self.l90 < 1 :
                move_no = 4
                if self.turn2(360):
                    return self.cluedo_flag
                self.move_to_point(center)
        return self.cluedo_flag

    #movement to find the the cluedo character in the green room
    def find_cluedo(self, center, mid):

        #move to center of room
        self.move_to_point(center)
        #scan 360 degrees from the center of the room
        if self.turn2(360):
            return self.cluedo_flag
            print ("cluedo_flag is true")
        rospy.sleep(1)

        print ("character cannot be seen from center of room")

        angle_to_spin = random.randint(-180,179)

        while self.cluedo_flag == False:
            # choose random direction and turn

            print(" TURN 1: {} ".format(angle_to_spin))
            self.turn2(angle_to_spin)
            time_0 = rospy.Time.now().to_sec()

            while self.l90 > 1.5 or np.isnan(self.l90):
                # Move forward until we hit something
                move_twist = Twist()
                move_twist.linear.x = 1
                move_twist.angular.z = 0
                self.move_pub.publish(move_twist)
                time_1 = rospy.Time.now().to_sec()
                if time_1 - time_0 > 4:
                    robot = PointStamped()
                    robot.header.frame_id = "base_footprint"
                    robot.header.stamp = rospy.Time.now()
                    robot.point.x = 0
                    robot.point.y = 0

                    d = self.transform_to_basefootprint(mid)
                    dist = self.euclidean_dist(robot.point, d.point)

                    if dist < 3:
                        self.turn2(180)
                    self.turn2(360)
                    time_0 = rospy.Time.now().to_sec()

            move_stop = Twist()
            move_stop.linear.x = 0
            move_stop.angular.z = 0
            self.move_pub.publish(move_stop)
            rospy.sleep(1)
            # Set angle to spin to be in range -90 to -180 or 90 to 180
            pn = random.randint(0,1)
            if pn == 0:
                # -90 to -180
                angle_to_spin = random.randint(-180,-90)
            else:
                # 90 to 180
                angle_to_spin = random.randint(90,180)

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
        while(curr_angle < angle and self.cluedo_flag == False):
            self.move_pub.publish(turn)
            t1 = rospy.Time.now().to_sec()
            curr_angle = turn_speed * (t1-t0)
            #if self.cluedo_flag == True:
            #    return self.cluedo_flag

        return self.cluedo_flag


    def choose_door(self):
        if self.closest_point() == 1:
            #self.move_to_point(self.first_room.point)
            if self.find_cluedo(self.first_room.point, self.first_door):
                print("cluedo image found")
            else:
                print("cluedo image not found")
        else:
            #self.move_to_point(self.second_room.point)
            if self.find_cluedo(self.second_room.point, self.second_door):
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
    handler = SpawnHandler()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down find_cleudo")



if __name__ == "__main__":
    main(sys.argv)
