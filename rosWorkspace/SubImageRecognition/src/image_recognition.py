#!/usr/bin/env python

import roslib
roslib.load_manifest("SubImageRecognition")
import sys
import rospy
import cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class image_converter:
    
    # Use these to enable/disable processing of forward and downward cameras
    forward = False
    downward = True
    
    def __init__(self):
        self._bridge = CvBridge()
        
        if forward:
            if downward:
                topic = "left/image_raw"
            else:
                topic = "image_raw"
            cv.NamedWindow("Forward Camera", cv.CV_WINDOW_AUTOSIZE)
            self._forward_pub = rospy.Publisher("forward_camera", Image)
            self._forward_sub = rospy.Subscriber(
                     topic, Image, self.forward_callback)
        
        if downward:
            if forward:
                topic = "right/image_raw"
            else:
                topic = "image_raw"
            cv.NamedWindow("Downward Camera", cv.CV_WINDOW_AUTOSIZE)
            self._downward_pub = rospy.Publisher("downward_camera", Image)
            self._downward_sub = rospy.Subscriber(
                    topic, Image, self.downward_callback)
    
    def forward_callback(self, data):
        try:
            cv_image = self._bridge.imgmsg_to_cv(data, "bgr8")
        except CvBridgeError, e:
            print e
        
        (cols, rows) = cv.GetSize(cv_image)
        gray = cv.CreateImage((cols, rows), cv.IPL_DEPTH_8U, 1)
        cv.CvtColor(cv_image, gray, cv.CV_RGB2GRAY)
        eig_image = cv.CreateMat(rows, cols, cv.CV_32FC1)
        temp_image = cv.CreateMat(rows, cols, cv.CV_32FC1)
        for (x, y) in cv.GoodFeaturesToTrack(gray, eig_image, temp_image, 10, 0.04, 1.0, useHarris=True):
            cv.Circle(cv_image, (int(x), int(y)), 1 , (0, 0, 255), 5)
        
        cv.ShowImage("Forward Camera", cv_image)
        cv.WaitKey(3)
        
        try:
            self._forward_pub.publish(self._bridge.cv_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError, e:
            print ed
    
    def downward_callback(self, data):
        try:
            cv_image = self._bridge.imgmsg_to_cv(data, "bgr8")
        except CvBridgeError, e:
            print e
        
        (cols, rows) = cv.GetSize(cv_image)
        gray = cv.CreateImage((cols, rows), cv.IPL_DEPTH_8U, 1)
        cv.CvtColor(cv_image, gray, cv.CV_RGB2GRAY)
        eig_image = cv.CreateMat(rows, cols, cv.CV_32FC1)
        temp_image = cv.CreateMat(rows, cols, cv.CV_32FC1)
        points = []
        for (x, y) in cv.GoodFeaturesToTrack(gray, eig_image, temp_image, 10, 0.04, 1.0, useHarris=True):
            points.append((int(x), int(y)))
        for point1 in points:
            for point2 in points:
                cv.Line(gray, point1, point2, 255, 2)
        
        cv.ShowImage("Downward Camera", gray)
        cv.WaitKey(3)
        
        try:
            self._downward_pub.publish(self._bridge.cv_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError, e:
            print ed


def main(args):
    ic = image_converter()
    rospy.init_node("image_recognition", anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv.DestroyAllWindows()


if __name__ == "__main__":
    main(sys.argv)

