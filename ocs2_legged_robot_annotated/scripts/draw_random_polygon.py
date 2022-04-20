#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PolygonStamped, Point32
from jsk_recognition_msgs.msg import PolygonArray
from std_msgs.msg import Header
import numpy as np
import scipy.io as sio


def ConvexPolygon(vertices, header):
    p = PolygonStamped()
    p.header = header
    nums = len(vertices)

    # z = -0.21
    z = -0.1

    for i in range(nums):
        p.polygon.points.append(Point32(vertices[i][0], vertices[i][1], z))
    
    return p


if __name__ == "__main__":
    # load polygons
    data = sio.loadmat("/home/parallels/ocs2_ws/src/ocs2_legged_robot_annotated/scripts/regions.mat")
    Nums = len(data['planes'][0])

    rospy.init_node("draw_polygon")
    pub = rospy.Publisher("polygon", PolygonArray, queue_size=10)
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        msg = PolygonArray()
        header = Header()
        header.frame_id = "odom"
        header.stamp = rospy.Time.now()
        msg.header = header

        for i in range(Nums):
            msg.polygons.append(ConvexPolygon(data['planes'][0][i][0][0][0], header))
            msg.labels.append(i)
            msg.likelihood.append(np.random.ranf())

        pub.publish(msg)
        r.sleep()
