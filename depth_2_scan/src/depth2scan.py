#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2
from sensor_msgs.msg import PointCloud2

logPointcloud = False

def horizontal_laser_scan_callback(msg):
    log_pointcloud_info("horizontal", msg)

def vertical_laser_scan_callback(msg):
    log_pointcloud_info("vertical", msg)

def depth_image_callback(msg):
    global logPointcloud
    if logPointcloud is False:
        log_pointcloud(msg)
        logPointcloud = True

    length = sensor_msgs.point_cloud2.read_points_list(msg, skip_nans=True)
    rospy.loginfo('%d' % len(length))

def log_pointcloud(msg):
    file = open("/home/hcchou/points.txt", "w")
    for point in sensor_msgs.point_cloud2.read_points(msg, skip_nans=True):
        str = '%.4f\t%.4f\t%.4f\t%.4f' % (point[0], point[1], point[2], point[3])
        file.write(str)
    file.close()

def log_pointcloud_info(label, msg):
    str = 'label: %s, height: %d, width: %d, fields: %d, row_step: %d, data: %d \n' % \
          (label, msg.height, msg.width, len(msg.fields), msg.row_step, len(msg.data))
    rospy.loginfo(str)

if __name__ == '__main__':
    rospy.init_node('depth2scan', anonymous=True)
    rospy.Subscriber("horizontal_laser_3d", PointCloud2, horizontal_laser_scan_callback)
    rospy.Subscriber("vertical_laser_3d", PointCloud2, vertical_laser_scan_callback)
    rospy.Subscriber("/zed2/zed_node/point_cloud/cloud_registered", PointCloud2, depth_image_callback)
    rospy.spin()