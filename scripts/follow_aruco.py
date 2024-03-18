#!/usr/bin/python

import rospy
import cv2
import collections
import numpy as np
from cv2 import aruco
import cv_bridge

from sensor_msgs.msg import Image
from exposure_control.msg import ExposureRegion

# configuration
TRACKED_MARKER_ID = 3
MISSING_MARKER_FRAMES = 10 # frames in a row until the exposure region becomes the whole frame by setting an invalid region
CAMERA_IMAGE_TOPIC = "/camera_array/cam1/image_raw"
IMAGE_WIDTH = 1440
IMAGE_HEIGHT = 1080

class ArucoExposureTracker:
    def __init__(self, tracked_marker_id):
        self.tracked_marker_id = tracked_marker_id
        self.exposure_region_publisher = rospy.Publisher("/target_region", ExposureRegion, queue_size=1)
        self.bridge = cv_bridge.CvBridge()
        self.image_subscriber = rospy.Subscriber(
            CAMERA_IMAGE_TOPIC,
            Image,
            self.image_callback,
            queue_size=1
        )
        self.num_frames_with_no_marker = 0
        self.marker_lost_published = False

    def publish_region(self, minx, maxx, miny, maxy):
        exposure_message = ExposureRegion()
        exposure_message.min_x = int(minx)
        exposure_message.min_y = int(miny)
        exposure_message.max_x = int(maxx)
        exposure_message.max_y = int(maxy)
        exposure_message.image_width = 1440
        exposure_message.image_height = 1080

        self.exposure_region_publisher.publish(exposure_message)

    def image_callback(self, image_msg):
        dictionary = aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        parameters =  aruco.DetectorParameters_create()
        #detector = aruco.ArucoDetector(dictionary)#, parameters)

        corners_by_id = collections.defaultdict(lambda: [])

        gray_img = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')
        #gray_img = cv2.cvtColor(gray_img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray_img, dictionary, parameters=parameters)

        for marker_id, marker_corners in zip(np.ravel(ids), corners):
            winSize = (3, 3)
            zeroZone = (-1, -1)
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TermCriteria_COUNT, 100, 0.0001)
            mcorners = cv2.cornerSubPix(gray_img, marker_corners, winSize, zeroZone, criteria)
            corners_by_id[marker_id] = corners_by_id[marker_id] + [mcorners]

        if self.tracked_marker_id in corners_by_id.keys():
            corners = corners_by_id[self.tracked_marker_id][0][0]
            xs = [x[0] for x in corners]
            ys = [x[1] for x in corners]
            self.publish_region(min(xs), max(xs), min(ys), max(ys))
            self.marker_lost_published = False
            self.num_frames_with_no_marker = 0
        else:
            self.num_frames_with_no_marker = self.num_frames_with_no_marker + 1
            if (self.num_frames_with_no_marker >= MISSING_MARKER_FRAMES) and not self.marker_lost_published:
                rospy.logwarn("Tracked marker lost. Setting default exposure")
                self.publish_region(0, 0, 0, 0)

                if self.num_frames_with_no_marker >= MISSING_MARKER_FRAMES + 3:
                    self.marker_lost_published = True

if __name__ == '__main__':
    rospy.init_node('aruco_tracking_example')
    tracker = ArucoExposureTracker(tracked_marker_id=TRACKED_MARKER_ID)
    rospy.spin()