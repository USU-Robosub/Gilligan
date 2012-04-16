#!/usr/bin/env python

import roslib
roslib.load_manifest("SubImageRecognition")
import sys
import rospy
import math
import time
import cv
from settings import Settings
from algorithm import Algorithm
from heapq import nlargest
from std_msgs.msg import String
from sensor_msgs.msg import Image
from SubImageRecognition.msg import ImgRecObject
from cv_bridge import CvBridge, CvBridgeError


class ImageRecognition:
    forward_callback_counter = 0
    downward_callback_counter = 0
    
    def __init__(self):
        self._bridge = CvBridge()
        
        self._forward_img_pub = rospy.Publisher("forward_camera/image_raw", Image)
        self._forward_sub = rospy.Subscriber(
                "left/image_raw", Image, self.forward_callback)
        
        self._downward_img_pub = rospy.Publisher("downward_camera/image_raw", Image)
        self._downward_sub = rospy.Subscriber(
                "right/image_raw", Image, self.downward_callback)
        
        self._rotated = None
        self._segmented = None
        self._threshold = None
        self._temp_threshold = None
    
    def get_rotated(self, size):
        if self._rotated is None:
            self._rotated = cv.CreateImage(size, cv.IPL_DEPTH_8U, 3)
        return self._rotated
    
    def get_segmented(self, size):
        if self._segmented is None:
            self._segmented = cv.CreateImage(size, cv.IPL_DEPTH_8U, 3)
        return self._segmented
    
    def get_threshold(self, size):
        if self._threshold is None:
            self._threshold = cv.CreateImage(size, cv.IPL_DEPTH_8U, 1)
        return self._threshold
    
    def get_temp_threshold(self, threshold):
        if self._temp_threshold is None:
            self._temp_threshold = cv.CreateImage(cv.GetSize(threshold), cv.IPL_DEPTH_8U, 1)
        cv.Copy(threshold, self._temp_threshold)
        return self._temp_threshold
    
    def forward_callback(self, data):
        # Get image
        try:
            image_raw = self._bridge.imgmsg_to_cv(data, "bgr8")
        except CvBridgeError, e:
            print e
        
        # Rotate so that 'up' is the top of the sub
        size = (image_raw.height, image_raw.width)
        rotated = self.get_rotated(size)
        self.rotate_image_ccw(image_raw, rotated)
        
        # Segment image into HSV channels
        segmented = self.get_segmented(size)
        cv.CvtColor(rotated, segmented, cv.CV_BGR2HSV)
        
        # Access threshold object for possible use
        threshold = self.get_threshold(size)
        
        for algorithm in Settings.algorithms:
            
            if algorithm.enabled and algorithm.camera is Algorithm.FORWARD:
                
                for threshold_name in algorithm.thresholds:
                    
                    # Perform threshold to isolate a range of colors
                    threshold_values = algorithm.thresholds[threshold_name]
                    cv.InRangeS(segmented, threshold_values[0], threshold_values[1], threshold)
                    
                    self.reduce_noise(threshold)
                    
                    point_sets = self.sample_points(threshold, size, self.forward_callback_counter)
                    
                    self.publish_points(algorithm, point_sets, rotated, name=threshold_name)
        
        self.forward_callback_counter = (self.forward_callback_counter + 1) % Settings.sample_size
        
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
        
        # Access threshold object for possible use
        threshold = self.get_threshold(size)
        
        for algorithm in Settings.algorithms:
            
            if algorithm.enabled and algorithm.camera is Algorithm.DOWNWARD:
                
                for threshold_name in algorithm.thresholds:
                    
                    # Perform threshold to isolate a range of colors
                    threshold_values = algorithm.thresholds[threshold_name]
                    cv.InRangeS(segmented, threshold_values[0], threshold_values[1], threshold)
                    
                    self.reduce_noise(threshold)
                    
                    point_sets = self.sample_points(threshold, size, self.downward_callback_counter)
                    
                    self.publish_points(algorithm, point_sets, rotated, name=threshold_name)
        
        self.downward_callback_counter = (self.forward_callback_counter + 1) % Settings.sample_size
        
        # Publish image
        try:
            self._downward_img_pub.publish(self._bridge.cv_to_imgmsg(rotated, "bgr8"))
        except rospy.ROSException:
            # Generally this exception occurs if ROS wasn't ready yet. We'll
            # just silently ignore it and next time should work
            pass
        except CvBridgeError, e:
            print e
    
    def publish_points(self, algorithm, point_sets, image, name):
        # Limit number of sets to publish based on algorithm
        point_sets = nlargest(algorithm.max_point_sets, point_sets, len)
        
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
                
                # Calculate confidence based on algorithm
                if algorithm.confidence_type is Algorithm.RECTANGLE:
                    expected_points = (dims[0] * dims[1]) / (Settings.sample_size ** 2)
                elif algorithm.confidence_type is Algorithm.CIRCLE:
                    expected_points = (math.pi * dims[0] * dims[1]) / (4 * Settings.sample_size ** 2)
                confidence = min(len(points) / expected_points, 1)
                
                # Publish object data
                algorithm.publisher.publish(ImgRecObject(
                    stamp = roslib.rostime.Time(time.time()),
                    name = name,
                    center_x = int(center[0]),
                    center_y = int(center[1]),
                    rotation = rotation,
                    height = int(dims[0]),
                    width = int(dims[1]),
                    confidence = confidence
                ))
    
    def sample_points(self, threshold, size, offset):
        # Get a copy of the image so we can modify it
        image = self.get_temp_threshold(threshold)
        
        # Sample image for white points
        point_sets = []
        for i in range(offset, size[1], Settings.sample_size): # height
            for j in range(offset, size[0], Settings.sample_size): # width
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
            if len(point_sets[index]) < Settings.min_point_set_len:
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
            if j+Settings.sample_size < size[0] and cv.Get2D(image, i, j+Settings.sample_size)[0] == 255.0:
                points.append((j+Settings.sample_size, i))
                cv.Set2D(image, i, j+Settings.sample_size, 0)
            if j-Settings.sample_size >= 0 and cv.Get2D(image, i, j-Settings.sample_size)[0] == 255.0:
                points.append((j-Settings.sample_size, i))
                cv.Set2D(image, i, j-Settings.sample_size, 0)
            if i+Settings.sample_size < size[1] and cv.Get2D(image, i+Settings.sample_size, j)[0] == 255.0:
                points.append((j, i+Settings.sample_size))
                cv.Set2D(image, i+Settings.sample_size, j, 0)
            if i-Settings.sample_size >= 0 and cv.Get2D(image, i-Settings.sample_size, j)[0] == 255.0:
                points.append((j, i-Settings.sample_size))
                cv.Set2D(image, i-Settings.sample_size, j, 0)
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

