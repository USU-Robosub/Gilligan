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
from SubImageRecognition.srv import ListAlgorithms, ListAlgorithmsResponse
from SubImageRecognition.srv import SwitchAlgorithm, SwitchAlgorithmResponse
from cv_bridge import CvBridge, CvBridgeError


class ImageRecognition:
    """
    A container class for doing all the heavy lifting needed for image
    recognition. It is configured from the static Settings class in settings.py
    using instances of the Algorithm class in algorithm.py.
    
    This class subscribes to the two image_raw topics published by the
    SubCameraDriver node and has separate but similar callbacks for each. It
    also has callbacks for the list_algorithms and switch_algorithm services.
    """
    
    def __init__(self):
        """
        Initialization function that sets up all subscriptions and services.
        Publishers for the annotated images are created here along with a
        CvBridge and 4 class variables used to reference persistent image memory
        """
        
        self._bridge = CvBridge()
        
        self._forward_callback_counter = 0
        self._downward_callback_counter = 0
        
        self._forward_img_pub = rospy.Publisher("forward_camera/image_raw", Image)
        rospy.Subscriber("left/image_raw", Image, self._forward_callback)
        
        self._downward_img_pub = rospy.Publisher("downward_camera/image_raw", Image)
        rospy.Subscriber("right/image_raw", Image, self._downward_callback)
        
        rospy.Service(Settings.ROOT_TOPIC + "list_algorithms",
                ListAlgorithms, self._list_algorithms_callback)
        
        rospy.Service(Settings.ROOT_TOPIC + "switch_algorithm",
                SwitchAlgorithm, self._switch_algorithm_callback)
        
        self._rotated = None
        self._segmented = None
        self._threshold = None
        self._temp_threshold = None
    
    def _get_rotated(self, size):
        """
        A getter for the rotated image memory. The memory is allocated on first
        access after the size of the image is known
        """
        
        if self._rotated is None:
            self._rotated = cv.CreateImage(size, cv.IPL_DEPTH_8U, 3)
        return self._rotated
    
    def _get_segmented(self, size):
        """
        A getter for the segmented image memory. The memory is allocated on
        first access after the size of the image is known
        """
        
        if self._segmented is None:
            self._segmented = cv.CreateImage(size, cv.IPL_DEPTH_8U, 3)
        return self._segmented
    
    def _get_threshold(self, size):
        """
        A getter for the threshold image memory. The memory is allocated on
        first access after the size of the image is known
        """
        
        if self._threshold is None:
            self._threshold = cv.CreateImage(size, cv.IPL_DEPTH_8U, 1)
        return self._threshold
    
    def _get_temp_threshold(self, threshold):
        """
        A getter for the temporary threshold image memory. The memory is
        allocated on first access after the size of the image is known. This
        method also copies the given threshold into the temporary one so that
        the temporary one can be modified without affecting the original
        """
        
        if self._temp_threshold is None:
            self._temp_threshold = cv.CreateImage(cv.GetSize(threshold), cv.IPL_DEPTH_8U, 1)
        cv.Copy(threshold, self._temp_threshold)
        return self._temp_threshold
    
    def _list_algorithms_callback(self, request):
        """
        Callback for the list_algorithms service. Just uses a list
        comprehension to quickly respond. I guess we could save the response
        in a class variable if this gets called a lot
        """
        
        return ListAlgorithmsResponse([algorithm.name for algorithm in Settings.ALGORITHMS])
    
    def _switch_algorithm_callback(self, request):
        """
        Callback for the switch_algorithm service. Searches all algorithms
        for one with a matching name and sets the enabled variable accordingly.
        Responds with status 1 on success or 0 on failure
        """
        
        for algorithm in Settings.ALGORITHMS:
            if algorithm.name == request.algorithm:
                algorithm.enabled = bool(request.enabled)
                return SwitchAlgorithmResponse(1)
        return SwitchAlgorithmResponse(0)
    
    def _forward_callback(self, data):
        """
        Callback for the forward camera subscription. Converts, rotates, and
        segments the incoming image. Then runs all applicable algorithms on it
        (each publishes to its own topic) and publishes an annotated image
        """
        
        # Get image
        try:
            image_raw = self._bridge.imgmsg_to_cv(data, "bgr8")
        except CvBridgeError, e:
            print e
        
        # Rotate so that 'up' is the top of the sub
        size = (image_raw.height, image_raw.width)
        rotated = self._get_rotated(size)
        self._rotate_image_ccw(image_raw, rotated)
        
        # Segment image into HSV channels
        segmented = self._get_segmented(size)
        cv.CvtColor(rotated, segmented, cv.CV_BGR2HSV)
        
        # Access threshold object for possible use
        threshold = self._get_threshold(size)
        
        for algorithm in Settings.ALGORITHMS:
            
            if algorithm.enabled and algorithm.camera is Algorithm.FORWARD:
                
                for threshold_name in algorithm.thresholds:
                    
                    # Perform threshold to isolate a range of colors
                    threshold_values = algorithm.thresholds[threshold_name]
                    cv.InRangeS(segmented, threshold_values[0], threshold_values[1], threshold)
                    
                    self._reduce_noise(threshold)
                    
                    point_sets = self._sample_points(threshold, size, self._forward_callback_counter)
                    
                    self._publish_points(algorithm, point_sets, rotated, name=threshold_name)
        
        self._forward_callback_counter = (self._forward_callback_counter + 1) % Settings.SAMPLE_SIZE
        
        # Publish image
        try:
            self._forward_img_pub.publish(self._bridge.cv_to_imgmsg(rotated, "bgr8"))
        except rospy.ROSException:
            # Generally this exception occurs if ROS wasn't ready yet. We'll
            # just silently ignore it and next time should work
            pass
        except CvBridgeError, e:
            print e
    
    def _downward_callback(self, data):
        """
        Callback for the downward camera subscription. Converts, rotates, and
        segments the incoming image. Then runs all applicable algorithms on it
        (each publishes to its own topic) and publishes an annotated image
        """
        
        # Get image
        try:
            image_raw = self._bridge.imgmsg_to_cv(data, "bgr8")
        except CvBridgeError, e:
            print e
        
        # Rotate so that 'up' is the front of the sub
        size = (image_raw.height, image_raw.width)
        rotated = self._get_rotated(size)
        self._rotate_image_ccw(image_raw, rotated)
        
        # Segment image into HSV channels
        segmented = self._get_segmented(size)
        cv.CvtColor(rotated, segmented, cv.CV_BGR2HSV)
        
        # Access threshold object for possible use
        threshold = self._get_threshold(size)
        
        for algorithm in Settings.ALGORITHMS:
            
            if algorithm.enabled and algorithm.camera is Algorithm.DOWNWARD:
                
                for threshold_name in algorithm.thresholds:
                    
                    # Perform threshold to isolate a range of colors
                    threshold_values = algorithm.thresholds[threshold_name]
                    cv.InRangeS(segmented, threshold_values[0], threshold_values[1], threshold)
                    
                    self._reduce_noise(threshold)
                    
                    point_sets = self._sample_points(threshold, size, self._downward_callback_counter)
                    
                    self._publish_points(algorithm, point_sets, rotated, name=threshold_name)
        
        self._downward_callback_counter = (self._downward_callback_counter + 1) % Settings.SAMPLE_SIZE
        
        # Publish image
        try:
            self._downward_img_pub.publish(self._bridge.cv_to_imgmsg(rotated, "bgr8"))
        except rospy.ROSException:
            # Generally this exception occurs if ROS wasn't ready yet. We'll
            # just silently ignore it and next time should work
            pass
        except CvBridgeError, e:
            print e
    
    def _publish_points(self, algorithm, point_sets, image, name):
        """
        Used by both image callbacks to publish details about the sets of
        points found. Also add annotations to the given image
        """
        
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
                    expected_points = (dims[0] * dims[1]) / (Settings.SAMPLE_SIZE ** 2)
                elif algorithm.confidence_type is Algorithm.CIRCLE:
                    expected_points = (math.pi * dims[0] * dims[1]) / (4 * Settings.SAMPLE_SIZE ** 2)
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
    
    def _sample_points(self, threshold, size, offset):
        """
        Looks for all contiguous regions of white points in an image by sampling
        a shifting grid of points and performing an exhaustive 'adjacent points'
        search when a white point is found. Regions with a low number of points
        are excluded from the returned list
        """
        
        # Get a copy of the image so we can modify it
        image = self._get_temp_threshold(threshold)
        
        # Sample image for white points
        point_sets = []
        for i in range(offset, size[1], Settings.SAMPLE_SIZE): # height
            for j in range(offset, size[0], Settings.SAMPLE_SIZE): # width
                if cv.Get2D(image, i, j)[0] == 255.0:
                    # Is this point in a point set already
                    found = False
                    for point_set in point_sets:
                        if (j, i) in point_set:
                            found = True
                            break
                    if not found:
                        # No so create new set from adjacent points and add to list
                        point_sets.append(self._find_adjacent_points(image, size, j, i))
        
        # Throw away bad point sets
        for index in range(len(point_sets)):
            if len(point_sets[index]) < Settings.MIN_POINTS:
                point_sets[index] = []
        
        return point_sets
    
    def _reduce_noise(self, image):
        """
        Applys a number of erode and dialate filters on the given image to
        reduce the amount of noise before sampling is performed
        """
        
        # Reduce noise in image using erode and dilate
        element = cv.CreateStructuringElementEx(3, 3, 1, 1, cv.CV_SHAPE_ELLIPSE)
        cv.Erode(image, image, element, 2)
        cv.Dilate(image, image, element, 4)
        element = cv.CreateStructuringElementEx(3, 3, 1, 1, cv.CV_SHAPE_RECT)
        cv.Erode(image, image, element, 2)
        cv.Dilate(image, image, element, 4)
    
    def _find_adjacent_points(self, image, size, j, i):
        """
        Peforms the exhaustive 'adjacent points' search mentioned in
        _sample_points(). The given image is modified during the search to
        quickly exclude added points from being found again
        """
        
        index = 0
        points = [(j, i)]
        cv.Set2D(image, i, j, 0)
        while index < len(points):
            j, i = points[index]
            if j+Settings.SAMPLE_SIZE < size[0] and cv.Get2D(image, i, j+Settings.SAMPLE_SIZE)[0] == 255.0:
                points.append((j+Settings.SAMPLE_SIZE, i))
                cv.Set2D(image, i, j+Settings.SAMPLE_SIZE, 0)
            if j-Settings.SAMPLE_SIZE >= 0 and cv.Get2D(image, i, j-Settings.SAMPLE_SIZE)[0] == 255.0:
                points.append((j-Settings.SAMPLE_SIZE, i))
                cv.Set2D(image, i, j-Settings.SAMPLE_SIZE, 0)
            if i+Settings.SAMPLE_SIZE < size[1] and cv.Get2D(image, i+Settings.SAMPLE_SIZE, j)[0] == 255.0:
                points.append((j, i+Settings.SAMPLE_SIZE))
                cv.Set2D(image, i+Settings.SAMPLE_SIZE, j, 0)
            if i-Settings.SAMPLE_SIZE >= 0 and cv.Get2D(image, i-Settings.SAMPLE_SIZE, j)[0] == 255.0:
                points.append((j, i-Settings.SAMPLE_SIZE))
                cv.Set2D(image, i-Settings.SAMPLE_SIZE, j, 0)
            index += 1
        return points
    
    def _rotate_image_ccw(self, image, rotated):
        """
        Copies a counter-clockwise version of 'image' into 'rotated' and returns it
        """
        
        cv.Transpose(image, rotated)
        cv.Flip(rotated, rotated, flipMode=0)
        return rotated
    
    def _rotate_image_cw(self, image, rotated):
        """
        Copies a clockwise version of 'image' into 'rotated' and returns it
        """
        
        cv.Transpose(image, rotated)
        cv.Flip(rotated, rotated, flipMode=1)
        return rotated


def main(args):
    """
    Executed when this file is directly called. Creates an instance of the
    ImageRecognition class and some ROS stuff
    """
    
    ir = ImageRecognition()
    rospy.init_node("image_recognition", anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"


if __name__ == "__main__":
    main(sys.argv)

