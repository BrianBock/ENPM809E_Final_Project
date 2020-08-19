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
right_check_angle = 15
speed = .1


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
        rotation = euler_from_quaternion(rot)[2]
        # euler_from_quaternion returns any angle > 180 as angle-360
        if rotation < 0:
            rotation = rotation + 2 * math.pi
        # print("Rotation is {a}".format(a=math.degrees(rotation)))
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo("TF Exception")
        return

    return Point(*trans), rotation


def compute_dist(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def find_wall(scan_data, attempt):
    """"Use the laser to determine where the closest wall is. Average laser scans in 15 deg increments to avoid nearby
    obstacles """
    inc = 5  # the size of the angle clusters (degrees)
    increments = list(range(0, 360 + inc, inc))
    angles = {}  # a dictionary of the distances measured at each degree angle. Each dictionary key is an inc degree
    # cluster list of angles
    std_mean = {}
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
        std_mean[dist] = (np.std(angles[dist]), sum(angles[dist]) / len(angles[dist]))

    # Get the 5 best wall candidates
    best_walls = []  # a list of the best wall candidates, where each candidate is a tuple (cluster, std, mean)
    for i in range(5):
        good_wall_key = min(std_mean.items(), key=lambda x: x[1])[0]
        best_walls.append((good_wall_key, std_mean[good_wall_key][0], std_mean[good_wall_key][1]))
        std_mean.pop(good_wall_key)

    closest_walls = sorted(best_walls, key=lambda x: x[2])
    print(closest_walls)
    nearest_wall = closest_walls[attempt][0]
    # print(angles[nearest_wall])
    print("The nearest wall is at an angle {x} degrees from my current position".format(x=nearest_wall))
    nearest_wall = math.radians(nearest_wall)  # Later operations are done in radians

    return nearest_wall


def go_straight_to_goal(goal, clearance):
    # Code adapted from lecture 8 example code
    (position, rotation) = get_odom_data()
    last_rotation = 0
    goal_x, goal_y = goal[0], goal[1]
    print("Navigating to goal ({x}, {y})".format(x=goal_x, y=goal_y))
    velocity_msg.angular.z = 0.0
    pub.publish(velocity_msg)

    # rospy.logwarn('goal_x: {x}'.format(x=goal_x))
    # rospy.logwarn('goal_y: {y}'.format(y=goal_y))

    # distance_to_goal = compute_dist(position.x, position.y, goal_x, goal_y)
    # rospy.logwarn('distance: {d}'.format(d=distance_to_goal))
    data = scan_360()
    distance_to_goal = data.ranges[0] - clearance

    while distance_to_goal > 0.05:
        print("Trying to get to {f}".format(f=goal))
        (position, rotation) = get_odom_data()
        x_start = position.x
        y_start = position.y
        # distance_to_goal = compute_dist(position.x, position.y, goal_x, goal_y)
        data = scan_360()
        distance_to_goal = data.ranges[0] - clearance
        velocity_msg.linear.x = min(k_h_gain * distance_to_goal, .3)
        velocity_msg.angular.z = 0.0
        pub.publish(velocity_msg)
        rate.sleep()

    velocity_msg.linear.x = 0.0
    velocity_msg.angular.z = 0.0
    pub.publish(velocity_msg)
    return True


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
        rate.sleep()

    command_speed(0, 0)
    return True


def drive():
    command_speed(1, 0)
    rospy.sleep(1)
    command_speed(0, 0)


def turn(angle, linspeed=0.0):
    """Turn 'angle' radians"""
    command_speed(linspeed, 0)
    print("Turning {a} degrees".format(a=math.degrees(angle)))
    (position, rotation) = get_odom_data()
    print("Current orientation is " + str(math.degrees(rotation)))
    # print(math.degrees(rotation))
    goal_angle = rotation + angle
    if goal_angle >= 2*math.pi:
        goal_angle = goal_angle - 2*math.pi
    print("Goal orientation is " + str(math.degrees(goal_angle)))
    # if goal_angle > math.pi:
    #     goal_angle = -(math.pi - goal_angle)
    #     print("New goal angle is "+str(math.degrees(goal_angle)))

    while abs(goal_angle - rotation) > math.radians(1):
        # print(math.degrees(goal_angle-rotation))
        velocity_msg.angular.z = k_v_gain * (goal_angle - rotation)
        if velocity_msg.angular.z > 0:
            velocity_msg.angular.z = min(velocity_msg.angular.z, .5)
        else:
            velocity_msg.angular.z = max(velocity_msg.angular.z, -.5)
        pub.publish(velocity_msg)
        (position, rotation) = get_odom_data()
        rate.sleep()
    velocity_msg.angular.z = 0.0
    pub.publish(velocity_msg)
    (position, rotation) = get_odom_data()
    print("New orientation is " + str(math.degrees(rotation)))
    print("Done turning")
    rate.sleep()


def go_to_wall(angle):
    """Take the output from find_wall. Turn to that orientation and drive until the robot is a predefined distance
    away from the wall. Rotate so the wall is on the robot's right. When this is finished, the robot should be ready
    for 'hug_wall()' """

    # Rotate to correct angle
    turn(angle)
    print("Facing the wall")
    rospy.sleep(.25)

    scan = scan_360()
    front = scan.ranges[0]
    clearance = .2
    (position, rotation) = get_odom_data()
    goaly = math.sin(rotation) * (front - clearance) + position.y
    goalx = math.cos(rotation) * (front - clearance) + position.x
    print("Current position: {x},{y}".format(x=position.x, y=position.y))
    print("Goal position: {x},{y}".format(x=goalx, y=goaly))
    # go_to_goal((goalx,goaly))
    go_straight_to_goal((goalx, goaly), clearance)
    # while front > clearance:
    #     # use P controller here. Robot should slow as it approaches wall
    #     (position, rotation) = get_odom_data()
    #     velocity_msg.linear.x = min(k_h_gain * (front-clearance), .2)
    #     pub.publish(velocity_msg)
    #     scan = scan_360()
    #     front = scan.ranges[0]
    velocity_msg.linear.x = 0.0
    velocity_msg.angular.z = 0.0
    pub.publish(velocity_msg)
    # rotate 90 CCW
    turn(math.pi / 2)
    # Save position for obstacle check
    wall_start = get_odom_data()[0]
    return wall_start, clearance


def determine_angle(F, FR, R, BR):
    if R > .5:  # or FR > .5 or BR > .5:
        angle = -math.radians(30)
        print("Uh oh. I no longer have the wall close my my right")
        return angle
    if FR == BR:
        angle = 0
        return angle
    elif FR > BR:
        a = BR * math.sin(math.radians(right_check_angle))
        c = BR * math.cos(math.radians(right_check_angle))
        d = abs(R - c)
        e = math.sqrt(d ** 2 + a ** 2)
        angle = -math.asin(d / e)
        # if angle > math.pi:
        #     angle = angle - math.pi
    elif FR < BR:
        a = FR * math.sin(math.radians(right_check_angle))
        c = FR * math.cos(math.radians(right_check_angle))
        d = abs(R - c)
        e = math.sqrt(d ** 2 + a ** 2)
        angle = math.asin(d / e)
        # if angle > math.pi:
        #     angle = angle - math.pi

    print("a:{a}, c:{c}, d:{d}, e:{e}".format(a=a, c=c, d=d, e=e))
    print("Need to turn {a} degrees".format(a=math.degrees(angle)))
    # if angle - math.pi >= 0:
    #     angle = -(angle - math.pi)
    return angle


def command_speed(lin, ang=0):
    """Command the Turtlebot to a specified linear and angular speed"""
    velocity_msg.linear.x = lin
    velocity_msg.angular.z = ang
    pub.publish(velocity_msg)
    rate.sleep()


def hug_wall(wall_start, clearance):
    """ This is the core of the wall follower algorithm. The robot drives straight with the wall a preset distance to
    it's right. """
    print("Wall following time!")
    success = False
    while not success:
        FFL, F, FFR, FR, R, BR = right_scan()
        print("Distances:")
        print("Front {f}, FR {fr}, R {r}, BR {br}".format(f=F, fr=FR, r=R, br=BR))
        # direction = 1
        # angle_dist = clearance / math.cos(math.radians(15))  # +.1
        # if F < .4:  # collision ahead
        #     print("collision ahead")
        #     if FFL < .4 or FFR < .4:
        #         print("I'm headfirst in a wall. Reverse!")
        #         command_speed(-speed, 0)
        #         rospy.sleep(.25)
        #         turn(math.radians(90))
        #         command_speed(speed, 0)
        #     else:
        #         command_speed(0, 0)
        #         turn(math.radians(-5))
        #         command_speed(speed, 0)
        # Victory conditions
        if F == float('inf'):
            print("I can see the exit!")
            if FR == float('inf'):
                print("I'm almost there!")
                angle_dist = math.inf
            if BR == float('inf'):
                print("Victory!")
                success = True
                break

        command_speed(speed, 0)
        if F < 2*clearance and FFL < 2*clearance and FFR < 2*clearance:
            print("I think there's a wall in front of me")
            command_speed(0, 0)
            turn(math.pi/2)
            command_speed(speed, 0)

        if FR > BR:
            # Robot has veered left of the wall and must turn right slightly to compensate
            print("Adjusting to the right")
            velocity_msg.angular.z = -.1
        elif BR > FR:
            # Robot has veered right of the wall and must turn left slightly to compensate
            print("Adjusting to the left")
            velocity_msg.angular.z = .1
        pub.publish(velocity_msg)
        rate.sleep()

        # ang = determine_angle(F, FR, R, BR)
        # turn(ang)
        # command_speed(speed, 0)
        # if (FR - BR) > .1:
        #     turn(math.radians(1))
        #     command_speed(speed)
        # elif (BR - FR) > .1:
        #     turn(math.radians(-1))
        #     command_speed(speed)
        #
        # if FR > (clearance/math.cos(math.radians(right_check_angle))):  # or BR < (clearance/math.cos(math.radians(right_check_angle)))+.1:
        #     # Robot has veered left of the wall and must turn right slightly to compensate
        #     corrective_angle = math.radians(right_check_angle) - math.acos(clearance/FR)
        #     print("Too far left!")
        #     print("I'm off by {a} degrees. Correcting that now.".format(a=corrective_angle))
        #     turn(corrective_angle, speed)
        #     command_speed(speed, 0)
        # if BR > (clearance/math.cos(math.radians(right_check_angle))):  # +.1 or FR < (clearance/math.cos(math.radians(right_check_angle)))+.1:
        #     # Robot has veered right of the wall and must turn left slightly to compensate
        #     corrective_angle = math.acos(clearance / FR) - math.radians(right_check_angle)
        #     print("Too far right!")
        #     print("I'm off by {a} degrees. Correcting that now.".format(a=corrective_angle))
        #     turn(corrective_angle)
        #     command_speed(speed, 0)
        # while (FR > angle_dist) or (BR > angle_dist):
        #     if FR - BR > .1:
        #         # Robot has veered left of the wall and must turn right slightly to compensate
        #         direction = -1
        #
        #     if BR - FR > .1:
        #         # Robot has veered right of the wall (may soon collide) and must turn left slightly to compensate
        #         direction = 1
        # #
        # velocity_msg.angular.z = min(direction)

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

    return scan.ranges[15], scan.ranges[0], scan.ranges[360 - 15], scan.ranges[270 + right_check_angle], scan.ranges[
        270], scan.ranges[270 - right_check_angle]


# turn(math.radians(35))
# turn(math.radians(50))
# turn(math.radians(65))
# turn(math.radians(65))


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
        print("I made it out of the maze!")
        print("Exiting program")
        break
    else:
        go_to_goal(scan_start)
        attempt += 1
