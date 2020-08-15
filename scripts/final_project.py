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


def find_wall(scan_data, attempt):
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
    # The best wall contenders have small deviations. The closest of those will be the one with the smallest average
    # distance
    for dist in angles:
        std_dict[dist] = np.std(angles[dist])
        mean_dict[dist] = sum(angles[dist]) / len(angles[dist])

    nearest_wall = (min(std_dict.items(), key=lambda x: x[1])[0])
    # print(angles[nearest_wall])
    print("The nearest wall is at an angle {x} degrees from my current position".format(x=nearest_wall))
    nearest_wall = math.radians(nearest_wall)  # Later operations are done in radians

    return nearest_wall


def go_to_goal(goal):
    # Code adapted from lecture 8 example code
    (position, rotation) = get_odom_data()
    last_rotation = 0
    goal_x, goal_y = goal[0], goal[1]
    print("Navigating to goal ({x}, {y})".format(x=goal_x, y=goal_y))

    # rospy.logwarn('goal_x: {x}'.format(x=goal_x))
    # rospy.logwarn('goal_y: {y}'.format(y=goal_y))

    distance_to_goal = compute_dist(position.x, position.y, goal_x, goal_y)
    # rospy.logwarn('distance: {d}'.format(d=distance_to_goal))

    while distance_to_goal > 0.05:
        print("Trying to get to {f}".format(f=goal))
        (position, rotation) = get_odom_data()
        x_start = position.x
        y_start = position.y
        angle_to_goal = math.atan2(goal_y - y_start, goal_x - x_start)

        # The domain of arctan(x) is (-inf, inf)
        # We would like to restrict the domain to (0, 2pi)
        if angle_to_goal < -math.pi / 4 or angle_to_goal > math.pi / 4:
            if 0 > goal_y > y_start:
                angle_to_goal = -2 * math.pi + angle_to_goal
            elif 0 <= goal_y < y_start:
                angle_to_goal = 2 * math.pi + angle_to_goal
        if last_rotation > math.pi - 0.1 and rotation <= 0:
            rotation = 2 * math.pi + rotation
        elif last_rotation < -math.pi + 0.1 and rotation > 0:
            rotation = -2 * math.pi + rotation

        velocity_msg.angular.z = k_v_gain * (angle_to_goal - rotation)
        distance_to_goal = compute_dist(position.x, position.y, goal_x, goal_y)
        velocity_msg.linear.x = min(k_h_gain * distance_to_goal, .5)

        if velocity_msg.angular.z > 0:
            velocity_msg.angular.z = min(velocity_msg.angular.z, 1.5)
        else:
            velocity_msg.angular.z = max(velocity_msg.angular.z, -1.5)

        last_rotation = rotation
        pub.publish(velocity_msg)
        # read_scan()
        # go_to_goal(goal)
        rate.sleep()

    velocity_msg.linear.x = 0.0
    velocity_msg.angular.z = 0.0
    pub.publish(velocity_msg)
    return True


def drive():
    velocity_msg.linear.x = 1
    pub.publish(velocity_msg)
    rospy.sleep(1)
    velocity_msg.linear.x = 0
    pub.publish()


def turn(angle):
    """Turn 'angle' radians"""
    print("Turning {a} degrees".format(a=math.degrees(angle)))
    (position, rotation) = get_odom_data()
    # print(math.degrees(rotation))
    goal_angle = rotation + angle
    # print(math.degrees(angle_to_goal))

    while goal_angle - rotation > math.radians(1):
        velocity_msg.angular.z = k_v_gain * goal_angle - rotation
        if velocity_msg.angular.z > 0:
            velocity_msg.angular.z = min(velocity_msg.angular.z, 1)
        else:
            velocity_msg.angular.z = max(velocity_msg.angular.z, -1)
        pub.publish(velocity_msg)
        (position, rotation) = get_odom_data()
    velocity_msg.angular.z = 0.0
    pub.publish(velocity_msg)
    print("Done turning")


def go_to_wall(angle):
    """Take the output from find_wall. Turn to that orientation and drive until the robot is a predefined distance
    away from the wall. Rotate so the wall is on the robot's right. When this is finished, the robot should be ready
    for 'hug_wall()' """

    # Rotate to correct angle
    turn(angle)
    print("Facing the wall")

    scan = scan_360()
    front = scan.ranges[0]
    clearance = .2
    (position, rotation) = get_odom_data()
    goaly = math.sin(rotation) * (front - clearance) + position.y
    goalx = math.cos(rotation) * (front - clearance) + position.x
    print("Current position: {x},{y}".format(x=position.x, y=position.y))
    print("Goal position: {x},{y}".format(x=goalx, y=goaly))
    go_to_goal((goalx, goaly))
    # while front > clearance:
    #     # use P controller here. Robot should slow as it approaches wall
    #     (position, rotation) = get_odom_data()
    #     velocity_msg.linear.x = min(k_h_gain * (front-clearance), .2)
    #     pub.publish(velocity_msg)
    #     scan = scan_360()
    #     front = scan.ranges[0]
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)
    # rotate 90 CCW
    turn(math.pi / 2)
    # Save position for obstacle check
    wall_start = get_odom_data()[0]
    return wall_start, clearance


def hug_wall(wall_start, clearance):
    """ This is the core of the wall follower algorithm. The robot drives straight with the wall a preset distance to
    it's right. """
    print("Wall following time!")
    success = False
    velocity_msg.linear.x = .3
    pub.publish(velocity_msg)
    F, FR, R, BR = right_scan()
    print("Distances:")
    print("Front {f}, FR {fr}, R {r}, BR {br}".format(f=F, fr=FR, r=R, br=BR))
    direction = 1
    angle_dist = clearance / math.cos(math.radians(15))  # +.1
    theta = math.acos(R / FR)  # Angle between the horizontal and angle of FR
    phi = math.acos(R / BR)
    print("Theta is {t}".format(t=math.degrees(theta)))
    print("Phi is {p}".format(p=math.degrees(phi)))
    while (BR > angle_dist) or (FR > angle_dist):
        if FR - BR > .1:
            # Robot has veered left of the wall and must turn right slightly to compensate
            direction = -1

        if BR - FR > .1:
            # Robot has veered right of the wall (may soon collide) and must turn left slightly to compensate
            direction = 1

    velocity_msg.angular.z = min(direction)
    (position, rotation) = get_odom_data()
    if abs(position.x - wall_start.x) < 0.1 and abs(position.y - wall_start.y):
        # Robot has returned to where it started when it began following this wall. This 'wall' is an obstacle
        success = False
    return success


def scan_360():
    """Scan the entire environment (0-360deg) once"""
    print("Scanning!")
    scan_data = rospy.wait_for_message("scan", LaserScan, timeout=None)
    return scan_data


def right_scan():
    """Scan the laser. Return only the values needed for hug_wall"""
    scan = rospy.wait_for_message("scan", LaserScan, timeout=None)

    return scan.ranges[0], scan.ranges[285], scan.ranges[270], scan.ranges[255]


success = False
attempt = 0
velocity_msg.linear.x = 0
velocity_msg.angular.z = 0
pub.publish(velocity_msg)  # Ensure the robot starts at rest
while not success:
    if attempt >= 3:
        print("This is a hard maze! I'm going to drive a bit to see if that helps.")
        drive()
    if attempt >= 6:
        print("I've tried {a} times. I can't find a real wall and it's time to give up :(".format(a=attempt))
        print("Exiting program")
        break
    scan_data = scan_360()
    scan_start = get_odom_data()
    angle = find_wall(scan_data, attempt)
    wall_start, clearance = go_to_wall(angle)
    success = hug_wall(wall_start, clearance)
    if success:
        print("Made it out of the maze!")
        break
    else:
        go_to_goal(scan_start)
        attempt += 1
