#!/usr/bin/env python

import rospy
from statistics import mean


def find_wall():
    """Use the laser to determine where the closest wall is. Average laser scans in 15 deg increments to avoid nearby
    obstacles """
    increments = list(range(0, 375, 15))

    angles = {}
    for i in range(0, len(increments) - 1):
        my_list = []
        for q in range(increments[i], increments[i + 1] + 1):
            my_list.append(q)
        angles[increments[i]] = mean(my_list)

    nearest_wall = (min(angles.items(), key=lambda x: x[1])[0])
    print("The nearest wall is at an angle {x} degrees from my current position".format(x=nearest_wall))

    # rotate to correct angle

    while front > .3:
        # use P controller here. Robot should slow as it approaches wall
        drive_forward()
