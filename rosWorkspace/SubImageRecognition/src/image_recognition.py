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


# TODO: All values used (thresholds, max sets, etc.) should be kept in a
#       central configuration to allow for quick modification from one place.

class ImageRecognition:
    # Settings: These should be moved to a config file someday?
    sample_size = 6
    min_point_set_len = 30
    max_point_sets = 2
    
    forward_callback_counter = 0
    downward_callback_counter = 0
    
    def __init__(self):
        self._bridge = CvBridge()
        
        self._forward_gate_pub = rospy.Publisher("forward_camera/gate", OrangeRectangle)
        self._forward_img_pub = rospy.Publisher("forward_camera/image_raw", Image)
        self._forward_sub = rospy.Subscriber(
                 "left/image_raw", Image, self.forward_callback)
        
        self._downward_rect_pub = rospy.Publisher("downward_camera/orange_rectangle", OrangeRectangle)
        self._downward_img_pub = rospy.Publisher("downward_camera/image_raw", Image)
        self._downward_sub = rospy.Subscriber(
                "right/image_raw", Image, self.downward_callback)
    
    def forward_callback(self, data):
        # Get image
        try:
            image_raw = self._bridge.imgmsg_to_cv(data, "bgr8")
        except CvBridgeError, e:
            print e
        
        # Rotate so that 'up' is the top of the sub
        size = (image_raw.height, image_raw.width)
        rotated = cv.CreateImage(size, cv.IPL_DEPTH_8U, 3)
        self.rotate_image_cw(image_raw, rotated)
        
        # Segment image into HSV channels
        segmented = cv.CreateImage(size, cv.IPL_DEPTH_8U, 3)
        cv.CvtColor(rotated, segmented, cv.CV_BGR2HSV)
        
        # Perform threshold to isolate a range of colors
        threshold = cv.CreateImage(size, cv.IPL_DEPTH_8U, 1)
        cv.InRangeS(segmented, (165, 0, 0), (225, 255, 150), threshold)
        
        self.reduce_noise(threshold)
        
        point_sets = self.sample_points(threshold, size, self.forward_callback_counter)
        self.forward_callback_counter += 1
        if self.forward_callback_counter == self.sample_size:
            self.forward_callback_counter = 0
        
        # Limit maximum number of point sets
        point_sets = nlargest(self.max_point_sets, point_sets, len)
        
        self.publish_points(point_sets, rotated, self.forward_gate_callback)
        
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
        self.rotate_image_ccw(image_raw, rotated)
        
        # Segment image into HSV channels
        segmented = cv.CreateImage(size, cv.IPL_DEPTH_8U, 3)
        cv.CvtColor(rotated, segmented, cv.CV_BGR2HSV)
        
        # Perform threshold to isolate a range of colors
        threshold = cv.CreateImage(size, cv.IPL_DEPTH_8U, 1)
        cv.InRangeS(segmented, (5, 50, 50), (15, 255, 255), threshold)
        
        self.reduce_noise(threshold)
        
        point_sets = self.sample_points(threshold, size, self.downward_callback_counter)
        self.downward_callback_counter += 1
        if self.downward_callback_counter == self.sample_size:
            self.downward_callback_counter = 0
        
        # Limit maximum number of point sets
        point_sets = nlargest(self.max_point_sets, point_sets, len)
        
        self.publish_points(point_sets, rotated, self.downward_orange_rectangle_callback)
        
        # Publish image
        try:
            self._downward_img_pub.publish(self._bridge.cv_to_imgmsg(rotated, "bgr8"))
        except rospy.ROSException:
            # Generally this exception occurs if ROS wasn't ready yet. We'll
            # just silently ignore it and next time should work
            pass
        except CvBridgeError, e:
            print e
    
    def forward_gate_callback(self, center, rotation, dims, points_len):
        expected_points = (dims[0] * dims[1]) / (self.sample_size ** 2)
        confidence = min(points_len / expected_points, 1)
        self._forward_gate_pub.publish(OrangeRectangle(stamp=roslib.rostime.Time(time.time()), center_x=int(center[0]), center_y=int(center[1]), confidence=confidence))
    
    def downward_orange_rectangle_callback(self, center, rotation, dims, points_len):
        expected_points = (dims[0] * dims[1]) / (self.sample_size ** 2)
        confidence = min(points_len / expected_points, 1)
        self._downward_rect_pub.publish(OrangeRectangle(stamp=roslib.rostime.Time(time.time()), center_x=int(center[0]), center_y=int(center[1]), rotation=rotation, confidence=confidence))
    
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
                callback(center, rotation, dims, len(points))
    
    def sample_points(self, image, size, offset):
        # Sample image for white points
        point_sets = []
        for i in range(offset, size[1], self.sample_size): # height
            for j in range(offset, size[0], self.sample_size): # width
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
        cv.Set2D(image, i, j, 0)
        while index < len(points):
            j, i = points[index]
            if j+self.sample_size < size[0] and cv.Get2D(image, i, j+self.sample_size)[0] == 255.0:
                points.append((j+self.sample_size, i))
                cv.Set2D(image, i, j+self.sample_size, 0)
            if j-self.sample_size >= 0 and cv.Get2D(image, i, j-self.sample_size)[0] == 255.0:
                points.append((j-self.sample_size, i))
                cv.Set2D(image, i, j-self.sample_size, 0)
            if i+self.sample_size < size[1] and cv.Get2D(image, i+self.sample_size, j)[0] == 255.0:
                points.append((j, i+self.sample_size))
                cv.Set2D(image, i+self.sample_size, j, 0)
            if i-self.sample_size >= 0 and cv.Get2D(image, i-self.sample_size, j)[0] == 255.0:
                points.append((j, i-self.sample_size))
                cv.Set2D(image, i-self.sample_size, j, 0)
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

