#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import numpy as np
from threading import Thread

from tf.transformations import *

from geometry_msgs.msg import TwistStamped, PoseStamped

from std_srvs.srv import Trigger

goal_pub = rospy.Publisher("/goal", PoseStamped, queue_size=10)

n = 6

rospy.init_node('topic_parser')

pub = rospy.Publisher("/formations_generator/formation", String)
reform = rospy.ServiceProxy('/reform', Trigger)

startt = True

def st(m):
    global starting, startt
    starting.unregister()
    startt = False

starting = rospy.Subscriber('/goal', PoseStamped, st)

p = PoseStamped()

while startt:
    rospy.sleep(0.1)
rospy.sleep(10)
print("STARTTT")

while True:
    _, _, *central = rospy.wait_for_message("/path_generator/central", String).data.split()
    try:
        _, _, *walls = rospy.wait_for_message("/path_generator/walls", String, timeout=5).data.split()
    except:
        cent = list(map(float, central))
        p.pose.position.x = cent[3]
        p.pose.position.y = cent[4]
        p.pose.position.z = 0
        goal_pub.publish(p)
        break

    cent = list(map(float, central))
    print(cent)

    central = list(map(float, central))[-2:]
    walls = list(map(float, walls))
    a_walls = []
    all_result = []
    for i in range(0, len(walls), 4):
        cur = walls[i:i+4].copy()
        # cur[0] += central[0]
        cur[1] += central[1]
        print(cur)

        qty_w = int(cur[2] / 2.0)
        ws = [(((cur[2] / (qty_w*2)) + i*(cur[2]/qty_w)) - cur[2]/2 + cur[0]) * -1 for i in range(qty_w)]
        # print(qty_w, ws)
        ws = np.array(ws)

        qty_h = int(cur[3] / 1.5)
        hs = [((cur[3] / (qty_h * 2)) + i * (cur[3] / qty_h)) - cur[3] / 2 + cur[1] for i in range(qty_h)]
        # print(qty_h, hs)
        hs = np.array(hs)

        result = np.transpose([np.tile(ws, len(hs)), np.repeat(hs, len(ws))])
        # print(result)

        all_result += result.tolist()

        a_walls.append(cur)

    print(all_result)
    s = '0 w'
    for i in range(min(n, len(all_result))):
        s += ' ' + ' '.join(map(str, [0.0]+all_result[i]))

    print(s)

    t = Thread(target=reform)
    t.start()

    print('reform')

    r = rospy.Rate(10)
    end = rospy.get_rostime().secs + 10
    while rospy.get_rostime().secs < end:
        # print('publ')
        pub.publish(String(s))
        r.sleep()


    if len(cent) > 6:
        p.pose.position.x = cent[3]
        p.pose.position.y = cent[4]
        p.pose.position.z = 0

        # vec = [cent[3] - cent[0] + cent[6] - cent[3], cent[4] - cent[1] + cent[7] - cent[4]]
        if cent[1] + cent[7] > cent[4]-cent[1]:
            p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w = quaternion_from_euler(0,0,+np.pi/2)
        else:
            p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w = quaternion_from_euler(0, 0, -np.pi / 2)

        goal_pub.publish(p)
        rospy.sleep(10)
        print(euler_from_quaternion(
            [p.pose.orientation.w, p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z]))
        p.pose.position.x = cent[6] + (5 if -0.1 < euler_from_quaternion(
            [p.pose.orientation.w, p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z])[0] < 0.1 else 0)
        p.pose.position.y = cent[7] + (0 if -0.1 < euler_from_quaternion(
            [p.pose.orientation.w, p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z])[0] < 0.1 else 5)
        p.pose.position.z = 0
        print(p)
        goal_pub.publish(p)
        # povorot
    else:
        p.pose.position.x = cent[3] + (5 if -0.1 < euler_from_quaternion([p.pose.orientation.w, p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z])[0] < 0.1 else 0)
        p.pose.position.y = cent[4] + (0 if -0.1 < euler_from_quaternion([p.pose.orientation.w, p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z])[0] < 0.1 else 5)
        p.pose.position.z = 0
        goal_pub.publish(p)
    rospy.sleep(30)



    # print(central[-2:])
    # print(a_walls)


