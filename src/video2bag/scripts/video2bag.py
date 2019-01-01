#!/usr/bin/env python2

"""
Convert a video file to a bag file.

Ubuntu 18.04 and ROS Melodic ship with OpenCV 3, so the original code below requires some tuning.

References:
- https://stackoverflow.com/questions/31432870/ros-convert-video-file-to-bag
- http://wiki.ros.org/opencv3
- https://stackoverflow.com/questions/45215755/opencv-python-cv2-cv-cap-prop-fps-error
"""

from __future__ import print_function

import roslib

roslib.load_manifest('video2bag')
import sys
import rospy

from ros import rosbag

from cv_bridge import CvBridge
import cv2

from distutils.version import LooseVersion


# region Constants
OPENCV_IS_VERSION_2 = LooseVersion(cv2.__version__).version[0] == 2
OUTPUT_TOPIC = 'camera/image_raw'
# endregion


def create_video_bag(video_path, bagname):
    """Creates a bag file from a video file"""
    bag = rosbag.Bag(bagname, 'w')
    video_capture = cv2.VideoCapture(video_path)

    cb = CvBridge()
    if OPENCV_IS_VERSION_2:
        fps = video_capture.get(cv2.cv.CV_CAP_PROP_FPS)  # FIXME cv2.CV_CAP_PROP_FPS in OpenCV 2?
    else:
        fps = video_capture.get(cv2.CAP_PROP_FPS)
    if fps != fps or fps <= 1e-2:
        print("[WARN] Cannot get FPS from video file. Assuming 24.")
        fps = 24
    ret = True
    frame_id = 0
    while ret:
        ret, frame = video_capture.read()
        if not ret:
            break
        stamp = rospy.rostime.Time.from_sec(float(frame_id) / fps)
        frame_id += 1
        image = cb.cv2_to_imgmsg(frame, encoding='bgr8')
        image.header.stamp = stamp
        image.header.frame_id = "camera"
        bag.write(OUTPUT_TOPIC, image, stamp)
    video_capture.release()
    bag.close()


if __name__ == "__main__":
    if len(sys.argv) == 3:
        create_video_bag(*sys.argv[1:])
    else:
        print("Usage: video2bag video_filename bag_filename")
