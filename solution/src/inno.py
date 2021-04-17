#!/usr/bin/env python3
import sys

import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import PositionTarget, State, ExtendedState
from mavros_msgs.srv import SetMode, CommandBool, CommandVtolTransition, CommandHome

from threading import Thread

n = 6

rospy.init_node("innosharks")

setpoint_local_position_pub = [rospy.Publisher("/mavros{}/setpoint_position/local".format(i), PoseStamped, queue_size=10) for i in range(1, n+1)]
setpoint_local_position = [PoseStamped() for i in range(1, n+1)]

setmode = [rospy.ServiceProxy("/mavros{}/set_mode".format(i), SetMode) for i in range(1, n+1)]
arming = [rospy.ServiceProxy("/mavros{}/cmd/arming".format(i), CommandBool) for i in range(1, n+1)]

goal_pub = rospy.Publisher("/goal", PoseStamped, queue_size=10)


def send_position():
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        for i in range(n):
            setpoint_local_position_pub[i].publish(setpoint_local_position[i])
        r.sleep()


def start_thread():
    t = Thread(target=send_position)
    t.daemon = True
    t.start()


def takeoff(i):
    print("/mavros{}/local_position/pose".format(i+1))
    current_pose = rospy.wait_for_message("/mavros{}/local_position/pose".format(i+1), PoseStamped)
    setpoint_local_position[i].pose.position.x = current_pose.pose.position.x
    setpoint_local_position[i].pose.position.y = current_pose.pose.position.y
    setpoint_local_position[i].pose.position.z = 5.0
    setpoint_local_position[i].pose.orientation.x = 0.0
    setpoint_local_position[i].pose.orientation.y = 0.0
    setpoint_local_position[i].pose.orientation.z = 0.0
    setpoint_local_position[i].pose.orientation.w = 0.0

    rospy.sleep(0.5)

    setmode[i](custom_mode="OFFBOARD")
    arming[i](True)


def set_position(x,y,z):
    setpoint_local_position[0].pose.position.x = x
    setpoint_local_position[0].pose.position.y = y
    setpoint_local_position[0].pose.position.z = z


if __name__ == "__main__":
    start_thread()
    for i in range(n):
        print(i, "takeoff")
        takeoff(i)
        print('done')
        # print(setpoint_local_position)
        # rospy.sleep(10)
    rospy.sleep(1*n)
    p = PoseStamped()
    p.pose.position.x = 0.0
    p.pose.position.y = -72.0
    p.pose.position.z = 5.0
    goal_pub.publish(p)

