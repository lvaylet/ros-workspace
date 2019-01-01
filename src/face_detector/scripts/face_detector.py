#!/usr/bin/env python2

"""
Face detection with ROS and OpenCV. This example relies on cv_bridge to
convert the images between ROS and OpenCV, as messages are sent as Images.
For CompressedImage messages, numpy must be used.
"""

from __future__ import print_function

import numpy as np
# Python libs
import sys
import time
# OpenCV
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
# ROS libraries
import roslib
import rospy
# ROS messages
from sensor_msgs.msg import Image, CompressedImage

VERBOSE = True
USE_COMPRESSED_IMAGES = False  # determines whether to use Numpy or OpenCV Bridge

TOPIC_SUBSCRIBER = 'camera/image_raw'  # '/camera/image/compressed'
TOPIC_PUBLISHER = 'output/faces'


class FaceDetector:

    def __init__(self):
        """Initialize ROS publishers and subscribers"""
        message_type = CompressedImage if USE_COMPRESSED_IMAGES else Image
        # Topics to publish processed images to
        self.publisher_output = rospy.Publisher(TOPIC_PUBLISHER,
                                                message_type,
                                                queue_size=1)
        self.publisher_grayscale = rospy.Publisher('output/grayscale',
                                                   message_type,
                                                   queue_size=1)
        # Subscribed topic with raw images
        self.subscriber = rospy.Subscriber(TOPIC_SUBSCRIBER,
                                           message_type,
                                           self.callback,
                                           queue_size=1)
        if VERBOSE:
            print('Subscribed to: %s' % TOPIC_SUBSCRIBER)

        self.bridge = CvBridge()

        # Face detection
        self.face_cascade = cv.CascadeClassifier('haarcascade_frontalface_default.xml')
        self.eye_cascade = cv.CascadeClassifier('haarcascade_eye.xml')

    def callback(self, ros_data):
        """Callback function where images get converted and features detected"""
        if VERBOSE:
            print('Received data of type %s with size %dx%d' % (type(ros_data), ros_data.width, ros_data.height))

        img = None
        if USE_COMPRESSED_IMAGES:

            # Convert image to 1-D Numpy array
            np_array = np.fromstring(ros_data.data, dtype=np.uint8)  # returns a 1-D np.array with length=width*height*3
            # Save array for local debugging
            # Reload with:
            # >>> import numpy as np
            # >>> np_array = np.load('data/np_array.npy')
            # np.save('data/np_array.npy', np_array)

            # Reshape array

            if VERBOSE:
                print('Numpy Array: shape: %s, dtype: %s' % (np_array.shape, np_array.dtype))

            # FIXME The following imdecode returns None
            # Troubleshoot by loading the Numpy array serialized above. For example:
            # >>> import numpy as np
            # >>> import cv2 as cv
            # >>> np_array = np.load('data/np_array.npy')
            # >>> img = cv.imdecode(np_array, cv.IMREAD_COLOR)
            # >>> np_array = np_array.reshape(ros_data.height, ros_data.width, 3)
            # >>> ...
            # img = cv2.imdecode(np_arr, cv.CV_LOAD_IMAGE_COLOR)  # OpenCV < 3.0
            img = cv.imdecode(np_array, cv.IMREAD_COLOR)  # OpenCV >= 3.0

        else:

            # Convert image with OpenCV Bridge
            try:
                img = self.bridge.imgmsg_to_cv2(ros_data, 'bgr8')
            except CvBridgeError as e:
                print(e)

        if img is None:
            raise Exception('Could not load image!')

        # Convert to grayscale and publish
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        if gray is None:
            raise Exception('Could not convert image to grayscale!')
        try:
            self.publisher_grayscale.publish(self.bridge.cv2_to_imgmsg(gray, 'mono8'))
        except CvBridgeError as e:
            print(e)

        # Detect faces
        faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
        for (x, y, w, h) in faces:
            cv.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
            roi_gray = gray[y:y + h, x:x + w]
            roi_color = img[y:y + h, x:x + w]
            eyes = self.eye_cascade.detectMultiScale(roi_gray)
            for (ex, ey, ew, eh) in eyes:
                cv.rectangle(roi_color, (ex, ey), (ex + ew, ey + eh), (0, 255, 0), 2)

        # Display result. The imshow function should be followed by waitKey function which displays the image for
        # specified milliseconds. Otherwise, it won't display the image. For example, waitKey(0) will display the
        # window infinitely until any keypress (it is suitable for image display). waitKey(25) will display a frame
        # for 25 ms, after which display will be automatically closed.
        cv.imshow('img', img)
        cv.waitKey(delay=2)

        # Publish result
        if USE_COMPRESSED_IMAGES:
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = 'jpeg'
            msg.data = np.array(cv.imencode('.jpg', img)[1]).tostring()
            self.publisher_output.publish(msg)
        else:
            try:
                self.publisher_output.publish(self.bridge.cv2_to_imgmsg(img, 'bgr8'))
            except CvBridgeError as e:
                print(e)

        # self.subscriber.unregister()


def main(args):
    """Initializes and cleanup ROS node"""
    face_detector = FaceDetector()
    rospy.init_node('face_detector', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down...')
    cv.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
