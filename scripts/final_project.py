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
import numpy as np

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
    inc = 5  # the size of the angle clusters (degrees)
    increments = list(range(0, 360 + inc, inc))
    angles = {}  # a dictionary of the distances measured at each degree angle. Each dictionary key is an inc degree
    # cluster list of angles
    std_dict = {}
    mean_dict = {}
    for i in range(0, len(increments) - 1):
        my_list = []
        for q in range(increments[i], increments[i + 1]):
            my_list.append(scan_data.ranges[q])
        # print(i)
        # print(my_list)
        # print("\n")
        angles[increments[i]] = my_list  # sum(my_list) / len(my_list)
    # The closest object that is a wall will have the smallest std deviation and smallest mean
    for dist in angles:
        std_dict[dist] = np.std(angles[dist])
        mean_dict[dist] = sum(angles[dist]) / len(angles[dist])
    # print("Standard Deviations:")
    # print(std_dict)
    # print("Means")
    # print(mean_dict)

    nearest_wall = (min(std_dict.items(), key=lambda x: x[1])[0])
    # print(angles[nearest_wall])
    print("The nearest wall is at an angle {x} degrees from my current position".format(x=nearest_wall))
    nearest_wall = math.radians(nearest_wall)  # Later operations are done in radians

    return nearest_wall


def go_to_wall(angle):
    """Take the output from find_wall. Turn to that orientation and drive until the robot is a predefined distance
    away from the wall. Rotate so the wall is on the robot's right. When this is finished, the robot should be ready
    for 'hug_wall()' """

    # Rotate to correct angle
    (position, rotation) = get_odom_data()
    # print(math.degrees(rotation))
    # angle_to_goal = rotation + angle
    # print(math.degrees(angle_to_goal))

    while angle_to_goal - rotation > .008726646:  # half a degree, in radians
        velocity_msg.angular.z = k_v_gain * angle_to_goal - rotation
        if velocity_msg.angular.z > 0:
            velocity_msg.angular.z = min(velocity_msg.angular.z, 1.5)
        else:
            velocity_msg.angular.z = max(velocity_msg.angular.z, -1.5)
        pub.publish(velocity_msg)
        (position, rotation) = get_odom_data()
    velocity_msg.angular.z = 0.0
    pub.publish(velocity_msg)
    print("Facing the wall")

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


def scan_360():
    """Scan the entire environment (0-360deg) once"""
    print("Scanning!")
    scan_data = rospy.wait_for_message("scan", LaserScan, timeout=None)
    return scan_data


scan_data = scan_360()
angle = find_wall(scan_data)
go_to_wall(angle)
# hug_wall()
