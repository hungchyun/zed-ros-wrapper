#!/usr/bin/env python

import math
import numpy as np

import image_geometry
import rospy
import sensor_msgs.point_cloud2
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField

camera_info_ = None
camera_model_ = image_geometry.PinholeCameraModel()

pub_ = rospy.Publisher('/zed/zed_node/point_cloud/cloud_registered_modified', PointCloud2, queue_size=10)

logPointcloud = False

DUMMY_FIELD_PREFIX = '__'

type_mappings = [(PointField.INT8, np.dtype('int8')), (PointField.UINT8, np.dtype('uint8')), (PointField.INT16, np.dtype('int16')),
                 (PointField.UINT16, np.dtype('uint16')), (PointField.INT32, np.dtype('int32')), (PointField.UINT32, np.dtype('uint32')),
                 (PointField.FLOAT32, np.dtype('float32')), (PointField.FLOAT64, np.dtype('float64'))]

pftype_to_nptype = dict(type_mappings)

pftype_sizes = {PointField.INT8: 1, PointField.UINT8: 1, PointField.INT16: 2, PointField.UINT16: 2,
                PointField.INT32: 4, PointField.UINT32: 4, PointField.FLOAT32: 4, PointField.FLOAT64: 8}

def set_camera_info(camera_info):
    global camera_info_
    camera_info_ = camera_info

def get_camera_info():
    global camera_info_
    return camera_info_

def get_camara_model():
    global camera_model_
    return camera_model_

def fields_to_dtype(fields, point_step):
    '''Convert a list of PointFields to a numpy record datatype.
    '''
    offset = 0
    np_dtype_list = []
    for f in fields:
        while offset < f.offset:
            # might be extra padding between fields
            np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
            offset += 1

        dtype = pftype_to_nptype[f.datatype]
        if f.count != 1:
            dtype = np.dtype((dtype, f.count))

        np_dtype_list.append((f.name, dtype))
        offset += pftype_sizes[f.datatype] * f.count

    # might be extra padding between points
    while offset < point_step:
        np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
        offset += 1

    return np_dtype_list

def horizontal_laser_scan_callback(msg):
    log_pointcloud_info("horizontal", msg)

def vertical_laser_scan_callback(msg):
    log_pointcloud_info("vertical", msg)

def depth_image_callback(msg):

    np.array([0, 0, 0])

    camara_model = get_camara_model()
    for i in range(msg.height):
        for j in range(msg.width):
            depth = 10.0
            x = (j - camara_model.cx()) * depth / camara_model.fx()
            y = (i - camara_model.cy()) * depth / camara_model.fy()
            z = depth

    cloud_msg = PointCloud2()
    cloud_msg.header = msg.header
    cloud_msg.height = msg.height
    cloud_msg.width = msg.width
    cloud_msg.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                        PointField('y', 4, PointField.FLOAT32, 1),
                        PointField('z', 8, PointField.FLOAT32, 1),
                        PointField('rgb', 12, PointField.FLOAT32, 1)]
    cloud_msg.is_bigendian = False
    cloud_msg.is_dense = False


def point_cloud_callback(msg):

    limit_size = 300
    cloud_arr_filterd = np.zeros(shape=(1, 4), dtype=np.float32)

    dtype_list = fields_to_dtype(msg.fields, msg.point_step)

    # parse the cloud into an array
    cloud_arr = np.frombuffer(msg.data, dtype_list)

    # remove the dummy fields that were added
    cloud_arr = cloud_arr[
        [fname for fname, _type in dtype_list if not (fname[:len(DUMMY_FIELD_PREFIX)] == DUMMY_FIELD_PREFIX)]]

    # rgb = np.array((255 << 16) | (0 << 8) | (0 << 0), dtype=np.uint32)

        # if math.isnan(cloud_arr[ind][0]) is False and len(cloud_arr_filterd) < limit_size:
    for ind in range(len(cloud_arr)):
        if math.isnan(cloud_arr[ind][0]) is False and len(cloud_arr_filterd) < limit_size:
            cloud_arr_filterd = np.vstack((cloud_arr_filterd, np.array((cloud_arr[ind][0], cloud_arr[ind][1], cloud_arr[ind][2], cloud_arr[ind][3]), dtype=np.float32)))

    # cloud_arr_filterd = np.delete(cloud_arr_filterd, 0, 0)

    modified_msg = msg
    modified_msg.height = 1
    modified_msg.width = limit_size
    modified_msg.is_bigendian = False
    modified_msg.point_step = 16
    modified_msg.row_step = modified_msg.point_step * modified_msg.width
    modified_msg.data = cloud_arr_filterd.tostring()
    modified_msg.is_dense = False
    pub_.publish(modified_msg)

def camera_info_callback(msg):
    global camera_model_, camera_info_
    rospy.loginfo("Update camera info.")
    set_camera_info(msg)
    rospy.loginfo("Update camera model.")
    camera_model_.fromCameraInfo(msg)


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
    # rospy.Subscriber("horizontal_laser_3d", PointCloud2, horizontal_laser_scan_callback)
    # rospy.Subscriber("vertical_laser_3d", PointCloud2, vertical_laser_scan_callback)
    rospy.Subscriber("/zed/zed_node/point_cloud/cloud_registered", PointCloud2, point_cloud_callback)
    # rospy.Subscriber("/zed/zed_node/depth/depth_registered", Image, depth_image_callback())
    # rospy.Subscriber("/zed/zed_node/depth/camera_info", CameraInfo, camera_info_callback())

    rospy.spin()