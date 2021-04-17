from time import sleep
from math import pi
import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped
from tf.transformations import *

rospy.init_node("goal_publisher")
pub = rospy.Publisher("/goal", PoseStamped)

# Function to move by XY-axis


def moveXY(start, end, dt):

    def sign(n):
        return 1 if n >= 0 else -1

    xs, ys, zs, ans = start
    xe, ye, ze, ane = end

    dx = xe - xs
    dy = ye - ys

    dir_x = sign(dx)
    dir_y = sign(dy)

    for x in range(abs(dx)):
        xs += dir_x
        yield [xs, ys, zs, ane]
        sleep(dt)

    for y in range(abs(dy)):
        ys += dir_y
        yield [xs, ys, zs, ane]
        sleep(dt)


sp = [0, -72, 5, 0]
tr = [41, -72, 5, pi/2]
tl = [41, 72, 5, pi]
bl = [-41, 72, 5, 3 * pi / 2]
br = [-41, -72, 5, 2 * pi]

iterations = 3

for _ in range(iterations):
    p = PoseStamped()
    direction = 0
    delay = 0.3

    # from START to TOP-RIGHT corner
    for pos in moveXY(sp, tr, delay):
        p.pose.position.x = pos[0]
        p.pose.position.y = pos[1]
        p.pose.position.z = pos[2]
        direction = pos[3]
        pub.publish(p)

    p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w = quaternion_from_euler(
        0., 0., direction)
    pub.publish(p)

    # from TOP-RIGHT to TOP-LEFT corner
    for pos in moveXY(tr, tl, delay):
        p.pose.position.x = pos[0]
        p.pose.position.y = pos[1]
        p.pose.position.z = pos[2]
        direction = pos[3]
        pub.publish(p)

    p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w = quaternion_from_euler(
        0., 0., direction)
    pub.publish(p)

    # from TOP-LEFT to BOTTOM-LEFT corner
    for pos in moveXY(tl, bl, delay):
        p.pose.position.x = pos[0]
        p.pose.position.y = pos[1]
        p.pose.position.z = pos[2]
        direction = pos[3]
        pub.publish(p)

    p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w = quaternion_from_euler(
        0., 0., direction)
    pub.publish(p)

    # from BOTTOM-LEFT to BOTTOM-RIGHT corner
    for pos in moveXY(bl, br, delay):
        p.pose.position.x = pos[0]
        p.pose.position.y = pos[1]
        p.pose.position.z = pos[2]
        direction = pos[3]
        pub.publish(p)

    p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w = quaternion_from_euler(
        0., 0., direction)
    pub.publish(p)

    # from BOTTOM-RIGHT to START corner
    for pos in moveXY(br, sp, delay):
        p.pose.position.x = pos[0]
        p.pose.position.y = pos[1]
        p.pose.position.z = pos[2]
        direction = pos[3]
        pub.publish(p)

    p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w = quaternion_from_euler(
        0., 0., direction)
    pub.publish(p)
