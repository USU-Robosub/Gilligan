#!/usr/bin/env python

import roslib
roslib.load_manifest("SubImageRecognition")
import sys
import rospy
import math
import time
import cv
from heapq import nlargest
from std_msgs.msg import String
from sensor_msgs.msg import Image
from SubImageRecognition.msg import OrangeRectangle
from cv_bridge import CvBridge, CvBridgeError


class ImageRecognition:
    sample_size = 17
    min_point_set_len = 50
    max_point_sets = 2
    
    def __init__(self):
        self._bridge = CvBridge()
        
        #self._forward_data_pub = rospy.Publisher("forward_camera/data", TODO)
        self._forward_img_pub = rospy.Publisher("forward_camera/image_raw", Image)
        self._forward_sub = rospy.Subscriber(
                 "left/image_raw", Image, self.forward_callback)
        
        self._downward_rect_pub = rospy.Publisher("downward_camera/orange_rectangle", OrangeRectangle)
        self._downward_img_pub = rospy.Publisher("downward_camera/image_raw", Image)
        self._downward_sub = rospy.Subscriber(
                "left/image_raw", Image, self.downward_callback) # TODO: Change back to right at some point
    
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
        except rospy.ROSException:
            # Generally this exception occurs if ROS wasn't ready yet. We'll
            # just silently ignore it and next time should work
            pass
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
        self.rotate_image_cw(image_raw, rotated) # TODO: Change back to ccw at some point
        
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
        
        self.reduce_noise(threshold)
        
        point_sets = self.sample_points(threshold, size)
        
        # Limit maximum number of point sets
        point_sets = nlargest(self.max_point_sets, point_sets, len)
        
        callback = lambda center, rotation: self._downward_rect_pub.publish(OrangeRectangle(stamp=roslib.rostime.Time(time.time()), center_x=int(center[0]), center_y=int(center[1]), rotation=rotation))
        self.publish_points(point_sets, rotated, callback)
        
        # Publish image
        try:
            self._downward_img_pub.publish(self._bridge.cv_to_imgmsg(rotated, "bgr8"))
        except rospy.ROSException:
            # Generally this exception occurs if ROS wasn't ready yet. We'll
            # just silently ignore it and next time should work
            pass
        except CvBridgeError, e:
            print e
    
    def publish_points(self, point_sets, image, callback):
        # Find minimum area rotated rectangle around each set of points
        for points in point_sets:
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
                
                # Mark rectangle on image
                v0 = (dims[0]/2*math.cos(rotation), dims[0]/2*math.sin(rotation))
                cv.Circle(image, (int(center[0]), int(center[1])), 1 , (0, 0, 255), 5)
                cv.Line(image, (int(center[0]), int(center[1])), (int(center[0] + v0[0]), int(center[1] + v0[1])), (0, 0, 255))
                
                # Normalize rotation for publishing
                if rotation == 0:
                    rotation = math.pi * 2
                rotation -= 3 * math.pi / 2
                
                # Publish rectangle data
                callback(center, rotation)
    
    def sample_points(self, image, size):
        # Sample image for white points
        point_sets = []
        for i in range(0, size[1], self.sample_size): # height
            for j in range(0, size[0], self.sample_size): # width
                if cv.Get2D(image, i, j)[0] == 255.0:
                    # Is this point in a point set already
                    found = False
                    for point_set in point_sets:
                        if (j, i) in point_set:
                            found = True
                            break
                    if not found:
                        # No so create new set from adjacent points and add to list
                        point_sets.append(self.find_adjacent_points(image, size, j, i))
        
        # Throw away bad point sets
        for index in range(len(point_sets)):
            if len(point_sets[index]) < self.min_point_set_len:
                point_sets[index] = []
        
        return point_sets
    
    def reduce_noise(self, image):
        # Reduce noise in image using erode and dilate
        element = cv.CreateStructuringElementEx(3, 3, 1, 1, cv.CV_SHAPE_ELLIPSE)
        cv.Erode(image, image, element, 2)
        cv.Dilate(image, image, element, 4)
        element = cv.CreateStructuringElementEx(3, 3, 1, 1, cv.CV_SHAPE_RECT)
        cv.Erode(image, image, element, 2)
        cv.Dilate(image, image, element, 4)
    
    def find_adjacent_points(self, image, size, j, i):
        index = 0
        points = [(j, i)]
        while index < len(points):
            j, i = points[index]
            if j+self.sample_size < size[0] and cv.Get2D(image, i, j+self.sample_size)[0] == 255.0 and (j+self.sample_size, i) not in points:
                points.append((j+self.sample_size, i))
            if j-self.sample_size >= 0 and cv.Get2D(image, i, j-self.sample_size)[0] == 255.0 and (j-self.sample_size, i) not in points:
                points.append((j-self.sample_size, i))
            if i+self.sample_size < size[1] and cv.Get2D(image, i+self.sample_size, j)[0] == 255.0 and (j, i+self.sample_size) not in points:
                points.append((j, i+self.sample_size))
            if i-self.sample_size >= 0 and cv.Get2D(image, i-self.sample_size, j)[0] == 255.0 and (j, i-self.sample_size) not in points:
                points.append((j, i-self.sample_size))
            index += 1
        return points
    
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

