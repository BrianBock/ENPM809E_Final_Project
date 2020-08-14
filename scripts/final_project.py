#!/usr/bin/env python
"""ENPM809E Summer 2020 Final Project
Brian Bock"""

import rospy
import tf
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
import math

rospy.init_node("move_robot")
rate = rospy.Rate(4)  # 4 Hz
velocity_msg = Twist()
pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)
od = rospy.Subscriber("odom", Odometry)
tf_listener = tf.TransformListener()
odom_frame = 'odom'
base_frame = 'base_footprint'
k_h_gain = 1
k_v_gain = 1


def tf_first_time():
    # It seems the first time tf_listener is called, it fails, which is why this try is here
    try:
        tf_listener.waitForTransform(odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1))
        # base_frame = 'base_footprint'
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        try:
            tf_listener.waitForTransform(odom_frame, 'base_link', rospy.Time(), rospy.Duration(1))
            # base_frame = 'base_link'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
            rospy.signal_shutdown("tf Exception")


def get_odom_data():
    """Request the odom data from the odom topic"""
    # Code adapted from the Lecture 8 example"
    try:
        (trans, rot) = tf_listener.lookupTransform(odom_frame, base_frame, rospy.Time(0))
        rotation = euler_from_quaternion(rot)
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo("TF Exception")
        return

    return Point(*trans), rotation[2]


def compute_dist(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def find_wall(scan_data):
    """"Use the laser to determine where the closest wall is. Average laser scans in 15 deg increments to avoid nearby
    obstacles """
    increments = list(range(0, 375, 15))
    angles = {}
    for i in range(0, len(increments) - 1):
        my_list = []
        for q in range(increments[i], increments[i + 1]):
            my_list.append(scan_data.ranges[q])
        angles[increments[i]] = sum(my_list) / len(my_list)
    nearest_wall = (min(angles.items(), key=lambda x: x[1])[0])
    print(angles[nearest_wall])
    print("The nearest wall is at an angle {x} degrees from my current position".format(x=nearest_wall))

    return nearest_wall


def go_to_wall(angle):
    """Take the output from find_wall. Turn to that orientation and drive until the robot is a predefined distance
    away from the wall. Rotate so the wall is on the robot's right. When this is finished, the robot should be ready
    for 'hug_wall()' """

    # rotate to correct angle

    while front > .3:
        # use P controller here. Robot should slow as it approaches wall
        drive_forward()
        (position, rotation) = get_odom_data()
        distance_to_goal = compute_dist(position.x, position.y, goal_x, goal_y)
        velocity_msg.linear.x = min(k_h_gain * distance_to_goal, .2)


def hug_wall():
    """ This is the core of the wall follower algorithm. The robot drives straight with the wall a preset distance to
    it's right. """
    pass


def avoid_obstacle(scans):
    # left = scans[0]
    frontleft = scans[1]
    front = scans[2]
    frontright = scans[3]
    # right = scans[4]
    # speed = velocity_msg.linear.x  # Save the current speed for later
    # write code to do obstacle avoidance
    collision_ahead = False
    if front < .5 or frontleft < .2 or frontright < .2:
        collision_ahead = True
        print('Collision ahead!')
        rospy.logwarn('Collision ahead!')
        velocity_msg.linear.x = 0  # Stop
        velocity_msg.angular.z = 0
        pub.publish(velocity_msg)
        print(frontleft, frontright)
        if frontleft > frontright:
            # check left and right scan data. Turn towards emptiest side (furthest distance)
            velocity_msg.angular.z = 1.5
        else:
            velocity_msg.angular.z = -1.5
        pub.publish(velocity_msg)
        if front < .5 and frontleft < .2 and frontright < .2:
            print('Is this a wall?')
            velocity_msg.angular.z = 1.5
            pub.publish(velocity_msg)

    elif collision_ahead:
        velocity_msg.angular.z = 0
        velocity_msg.linear.x = .1
        pub.publish(velocity_msg)
    # else:
    #     collision_ahead = False


def sensor_callback(msg):
    front = msg.ranges[0]
    frontleft = msg.ranges[15]
    left = msg.ranges[90]
    frontright = msg.ranges[345]
    right = msg.ranges[270]

    rospy.loginfo("Distance from obstacle (front): {f}".format(f=front))
    rospy.loginfo("Distance from obstacle (left): {l}".format(l=left))
    rospy.loginfo("Distance from obstacle (right): {r}".format(r=right))

    avoid_obstacle([left, frontleft, front, frontright, right])


def read_scan():
    rospy.Subscriber("scan", LaserScan, sensor_callback)
    print("Done scanning")
    # rospy.spin()


# def callback_360(msg):

# return msg


def scan_360():
    """Scan the entire environment (0-360deg) once"""
    print("Scanning!")
    scan_data = rospy.wait_for_message("scan", LaserScan, timeout=None)
    return scan_data


scan_data = scan_360()
angle = find_wall(scan_data)
# go_to_wall(angle)
# hug_wall()
