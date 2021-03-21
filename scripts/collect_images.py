#! /home/jc/Envs/py36/bin/python3.6

import sys
import os
import rospy
import numpy as np
# from cv_bridge import CvBridge, CvBridgeError
# from cv_bridge.boost.cv_bridge_boost import getCvType
# if '/opt/ros/kinetic/lib/python2.7/dist-packages' in sys.path:
#     sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
# import cv2
from sensor_msgs.msg import Image


name_to_dtypes = {
	"rgb8":    (np.uint8,  3),
	"rgba8":   (np.uint8,  4),
	"rgb16":   (np.uint16, 3),
	"rgba16":  (np.uint16, 4),
	"bgr8":    (np.uint8,  3),
	"bgra8":   (np.uint8,  4),
	"bgr16":   (np.uint16, 3),
	"bgra16":  (np.uint16, 4),
	"mono8":   (np.uint8,  1),
	"mono16":  (np.uint16, 1),

}


def image_to_numpy(msg):
	if not msg.encoding in name_to_dtypes:
		raise TypeError('Unrecognized encoding {}'.format(msg.encoding))
	
	dtype_class, channels = name_to_dtypes[msg.encoding]
	dtype = np.dtype(dtype_class)
	print(msg.encoding)
	dtype = dtype.newbyteorder('>' if msg.is_bigendian else '<')
	shape = (msg.height, msg.width, channels)

	data = np.fromstring(msg.data, dtype=dtype).reshape(shape)
	data.strides = (
		msg.step,
		dtype.itemsize * channels,
		dtype.itemsize
	)

	if channels == 1:
		data = data[...,0]
	return data

rospy.init_node('camera_test')
rospy.loginfo("testing virtual env")

print('here')
data = rospy.wait_for_message('/camera/color/image_raw', Image)

np_image = image_to_numpy(data)

print(np_image.shape)