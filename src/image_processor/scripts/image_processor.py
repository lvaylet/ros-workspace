#!/usr/bin/env python2

"""OpenCV feature detection with ROS CompressedImage Topics in python.

This example subscribes to a ros topic containing sensor_msgs Image.
It converts the Image into a numpy.ndarray, then detects and marks
features in that image. It finally displays and publishes the new
image as a CompressedImage topic.
"""

from __future__ import print_function

# numpy and scipy
import numpy as np
from scipy.ndimage import filters
# Python libs
import sys
import time
# OpenCV
import cv2
# ROS libraries
import roslib
import rospy
# ROS messages
from sensor_msgs.msg import Image, CompressedImage

# We do not use cv_bridge as it does not support CompressedImage in python
# from cv_bridge import CvBridge, CvBridgeError

VERBOSE = False

TOPIC_SUBSCRIBER = 'camera/image_raw'  # '/camera/image/compressed'
TOPIC_PUBLISHER = 'output/image_raw/compressed'


class ImageFeature:

    def __init__(self):
        """Initialize ROS publishers and subscribers"""
        # Topic to publish to
        self.publisher = rospy.Publisher(TOPIC_PUBLISHER,
                                         CompressedImage,
                                         queue_size=1)
        # self.bridge = CvBridge()

        # Subscribed topic
        self.subscriber = rospy.Subscriber(TOPIC_SUBSCRIBER,
                                           Image,  # CompressedImage
                                           self.callback,
                                           queue_size=1)
        if VERBOSE:
            print('subscribed to: %s' % TOPIC_SUBSCRIBER)

    def callback(self, ros_data):
        """Callback function where images get converted and features detected"""
        if VERBOSE:
            print('received image of type: %s' % ros_data.format)

        # Direct conversion to CV2 #
        np_arr = np.fromstring(ros_data.data, np.uint8)
        # image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)  # OpenCV < 3.0
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0

        # Feature detection using CV2 #
        # "","Grid","Pyramid" + 
        # "FAST","GFTT","HARRIS","MSER","ORB","SIFT","STAR","SURF"
        # OpenCV < 3.0:
        # method = "GridFAST"
        # feature_detector = cv2.FeatureDetector_create(method)
        # OpenCV >= 3.0
        method = 'FAST'
        feature_detector = cv2.FastFeatureDetector_create()

        # Convert numpy image to grayscale and detect features
        time1 = time.time()
        grayscale_image = cv2.cvtColor(src=image_np, code=cv2.COLOR_BGR2GRAY)
        feature_points = feature_detector.detect(grayscale_image)
        time2 = time.time()
        if VERBOSE:
            print('%s detector found %s points in %s sec.' % (method, len(feature_points), time2 - time1))

        # Hightlight features with circles
        for featpoint in feature_points:
            x, y = featpoint.pt
            cv2.circle(img=image_np, center=(int(x), int(y)), radius=3, color=(0, 0, 255), thickness=-1)

        # cv2.imshow('cv_img', image_np)
        # cv2.waitKey(2)

        # Create and publish CompressedImage #
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
        self.publisher.publish(msg)

        # self.subscriber.unregister()


def main(args):
    """Initializes and cleanup ROS node"""
    # Instantiate an ImageFeature() object to trigger the registration of subscribers/publishers in __init__()
    image_feature = ImageFeature()
    rospy.init_node('image_processor', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down ROS Image Processor module')
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
