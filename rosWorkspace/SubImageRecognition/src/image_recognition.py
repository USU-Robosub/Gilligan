#!/usr/bin/env python

import roslib
roslib.load_manifest("SubImageRecognition")
import sys
import rospy
import math
import time
import cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from SubImageRecognition.msg import OrangeRectangle
from cv_bridge import CvBridge, CvBridgeError


class ImageRecognition:
    def __init__(self):
        self._bridge = CvBridge()
        
        #self._forward_data_pub = rospy.Publisher("forward_camera/data", TODO)
        self._forward_img_pub = rospy.Publisher("forward_camera/image_raw", Image)
        self._forward_sub = rospy.Subscriber(
                 "left/image_raw", Image, self.forward_callback)
        
        self._downward_rect_pub = rospy.Publisher("downward_camera/orange_rectangle", OrangeRectangle)
        self._downward_img_pub = rospy.Publisher("downward_camera/image_raw", Image)
        self._downward_sub = rospy.Subscriber(
                "left/image_raw", Image, self.downward_callback) # TODO: Change back to right/image_raw at some point
    
    def forward_callback(self, data):
        # Get image
        try:
            image_raw = self._bridge.imgmsg_to_cv(data, "bgr8")
        except CvBridgeError, e:
            print e
        
        # Rotate so that 'up' is the top of the sub
        rotated = cv.CreateImage((image_raw.height, image_raw.width), cv.IPL_DEPTH_8U, 3)
        self.rotate_image_cw(image_raw, rotated)
        
        # TODO: Right now this callback just runs 'good features to track'
        #       algorithm against a grayscale version of the rotated image.
        #       It needs to have a good idea of where it is in the pool (some
        #       other node should keep track of that and publish it) and then
        #       run different sets of image recognition algorithms based on
        #       what is expected to be around it.
        
        # Look for good features to track and mark them
        (cols, rows) = cv.GetSize(rotated)
        gray = cv.CreateImage((cols, rows), cv.IPL_DEPTH_8U, 1)
        cv.CvtColor(rotated, gray, cv.CV_RGB2GRAY)
        eig_image = cv.CreateMat(rows, cols, cv.CV_32FC1)
        temp_image = cv.CreateMat(rows, cols, cv.CV_32FC1)
        for (x, y) in cv.GoodFeaturesToTrack(gray, eig_image, temp_image, 10, 0.04, 1.0, useHarris=True):
            cv.Circle(rotated, (int(x), int(y)), 1 , (0, 0, 255), 5)
        
        # Publish image
        try:
            self._forward_img_pub.publish(self._bridge.cv_to_imgmsg(rotated, "bgr8"))
        except CvBridgeError, e:
            print e
    
    def downward_callback(self, data):
        # Get the image in OpenCV format via the bridge
        try:
            image_raw = self._bridge.imgmsg_to_cv(data, "bgr8")
        except CvBridgeError, e:
            print e
        
        # Rotate so that 'up' is the front of the sub
        size = (image_raw.height, image_raw.width)
        rotated = cv.CreateImage(size, cv.IPL_DEPTH_8U, 3)
        self.rotate_image_ccw(image_raw, rotated)
        
        # TODO: Right now this callback just looks for orange rectangles and
        #       published data about them. It needs to have a good idea of
        #       where it is (some other node should keep track of that and
        #       publish it) and then optionally run additional algorithms
        #       based on what is expected to be around it.
        
        # TODO: All values used in operations (thresholds, amount/type of
        #       erode/dilate, etc.) should be kept in a central configuration
        #       to allow for quick modification from one place.
        
        # Segment image into HSV channels
        segmented = cv.CreateImage(size, cv.IPL_DEPTH_8U, 3)
        cv.CvtColor(rotated, segmented, cv.CV_BGR2HSV)
        
        # Perform threshold to isolate a range of colors
        threshold = cv.CreateImage(size, cv.IPL_DEPTH_8U, 1)
        cv.InRangeS(segmented, (5, 50, 50), (15, 255, 255), threshold)
        
        # Reduce noise in threshold using erode and dilate
        element = cv.CreateStructuringElementEx(3, 3, 1, 1, cv.CV_SHAPE_ELLIPSE)
        cv.Erode(threshold, threshold, element, 2)
        cv.Dilate(threshold, threshold, element, 4)
        element = cv.CreateStructuringElementEx(3, 3, 1, 1, cv.CV_SHAPE_RECT)
        cv.Erode(threshold, threshold, element, 2)
        cv.Dilate(threshold, threshold, element, 4)
        
        # Sample threshold for white points
        # TODO: Segment points into grouped regions
        points = []
        for i in range(0, size[1], 13): # height
            for j in range(0, size[0], 13): # width
                if cv.Get2D(threshold, i, j)[0] == 255.0:
                    points.append((j, i))
        
        # Throw away bad groups
        # TODO: Do this for each of the grouped regions
        if len(points) < 5:
            points = None
        
        # Find minimum area rotated rectangle around white points
        #if points:
        if points:
            (center, dims, rotation) = cv.MinAreaRect2(points)
            rotation *= math.pi / 180 # Convert rotation from degrees to radians
            
            # Correct dims/rotation so that the first dim is always larger
            if dims[0] < dims[1]:
                dims = (dims[1], dims[0])
                rotation += math.pi / 2
            
            # Normalize rotation for drawing
            if math.sin(rotation) > 0:
                rotation += math.pi
            rotation %= math.pi * 2
            
            # Mark rectangle on rotated image
            v0 = (dims[0]/2*math.cos(rotation), dims[0]/2*math.sin(rotation))
            v1 = (dims[1]/2*math.cos(rotation+math.pi/2), dims[1]/2*math.sin(rotation+math.pi/2))
            cv.Circle(rotated, (int(center[0]), int(center[1])), 1 , (0, 0, 255), 5)
            cv.Line(rotated, (int(center[0]), int(center[1])), (int(center[0] + v0[0]), int(center[1] + v0[1])), (0, 0, 255))
            cv.Circle(rotated, (int(center[0] + v0[0] + v1[0]), int(center[1] + v0[1] + v1[1])), 1 , (0, 0, 255), 5)
            cv.Circle(rotated, (int(center[0] + v0[0] - v1[0]), int(center[1] + v0[1] - v1[1])), 1 , (0, 0, 255), 5)
            cv.Circle(rotated, (int(center[0] - v0[0] + v1[0]), int(center[1] - v0[1] + v1[1])), 1 , (0, 0, 255), 5)
            cv.Circle(rotated, (int(center[0] - v0[0] - v1[0]), int(center[1] - v0[1] - v1[1])), 1 , (0, 0, 255), 5)
            
            # Normalize rotation for publishing
            if rotation == 0:
                rotation = math.pi * 2
            rotation -= 3 * math.pi / 2
            
            # Publish rectangle data
            self._downward_rect_pub.publish(OrangeRectangle(stamp=roslib.rostime.Time(time.time()), center_x=int(center[0]), center_y=int(center[1]), rotation=rotation))
        
        # Publish image
        try:
            self._downward_img_pub.publish(self._bridge.cv_to_imgmsg(rotated, "bgr8"))
        except CvBridgeError, e:
            print e
    
    def rotate_image_ccw(self, image, rotated):
        cv.Transpose(image, rotated)
        cv.Flip(rotated, rotated, flipMode=0)
        return rotated
    
    def rotate_image_cw(self, image, rotated):
        cv.Transpose(image, rotated)
        cv.Flip(rotated, rotated, flipMode=1)
        return rotated


def main(args):
    ir = ImageRecognition()
    rospy.init_node("image_recognition", anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"


if __name__ == "__main__":
    main(sys.argv)

