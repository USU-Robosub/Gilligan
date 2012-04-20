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
from heapq import nlargest, heappush, heappop
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
        
        self._f_counter = 0
        self._d_counter = 0
        
        self._publishers = {}
        
        self._forward_img_pub = rospy.Publisher("forward_camera/image_raw", Image)
        rospy.Subscriber("left/image_raw", Image, self._forward_callback)
        
        self._downward_img_pub = rospy.Publisher("downward_camera/image_raw", Image)
        rospy.Subscriber("right/image_raw", Image, self._downward_callback)
        
        rospy.Service(Settings.ROOT_TOPIC + "list_algorithms",
                ListAlgorithms, self._list_algorithms_callback)
        
        rospy.Service(Settings.ROOT_TOPIC + "switch_algorithm",
                SwitchAlgorithm, self._switch_algorithm_callback)
        
        self._f_rotated = None
        self._d_rotated = None
        self._f_segmented = None
        self._d_segmented = None
        self._f_threshold = None
        self._d_threshold = None
        self._f_temp_threshold = None
        self._d_temp_threshold = None
    
    def _get_publisher(self, algorithm):
        """
        Builds a new publisher for an algorithm and saves it for later
        """
        
        name = algorithm.name
        if name not in self._publishers:
            self._publishers[name] = rospy.Publisher(Settings.ROOT_TOPIC + name, ImgRecObject)
        return self._publishers[name]
    
    def _init_image_memory(self, size):
        """
        Allocates memory for all the image class variables once we have a size
        to use
        """
        
        if self._d_temp_threshold is None:
            self._f_rotated = cv.CreateImage(size, cv.IPL_DEPTH_8U, 3)
            self._d_rotated = cv.CreateImage(size, cv.IPL_DEPTH_8U, 3)
            self._f_segmented = cv.CreateImage(size, cv.IPL_DEPTH_8U, 3)
            self._d_segmented = cv.CreateImage(size, cv.IPL_DEPTH_8U, 3)
            self._f_threshold = cv.CreateImage(size, cv.IPL_DEPTH_8U, 1)
            self._d_threshold = cv.CreateImage(size, cv.IPL_DEPTH_8U, 1)
            self._f_temp_threshold = cv.CreateImage(size, cv.IPL_DEPTH_8U, 1)
            self._d_temp_threshold = cv.CreateImage(size, cv.IPL_DEPTH_8U, 1)
    
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
        self._init_image_memory(size)
        rotated = self._f_rotated
        self._rotate_image_ccw(image_raw, rotated)
        
        # Segment image into HSV channels
        segmented = self._f_segmented
        cv.CvtColor(rotated, segmented, cv.CV_BGR2HSV)
        
        # Access threshold memory
        threshold = self._f_threshold
        temp_threshold = self._f_temp_threshold
        
        # Normalize image
        self._normalize(segmented, threshold)
        #cv.CvtColor(segmented, rotated, cv.CV_HSV2BGR)
        
        for algorithm in Settings.ALGORITHMS:
            
            if algorithm.enabled and algorithm.camera is Algorithm.Camera.FORWARD:
                
                for threshold_name in algorithm.thresholds:
                    
                    # Perform threshold to isolate a range of colors
                    threshold_values = algorithm.thresholds[threshold_name]
                    cv.InRangeS(segmented, threshold_values[0], threshold_values[1], threshold)
                    
                    self._reduce_noise(threshold)
                    
                    cv.Copy(threshold, temp_threshold)
                    point_sets = self._sample_points(temp_threshold, size, self._f_counter)
                    
                    self._publish_points(algorithm, point_sets, rotated, size, name=threshold_name)
        
        self._f_counter = (self._f_counter + 1) % Settings.SAMPLE_SIZE
        
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
        self._init_image_memory(size)
        rotated = self._d_rotated
        self._rotate_image_ccw(image_raw, rotated)
        
        # Segment image into HSV channels
        segmented = self._d_segmented
        cv.CvtColor(rotated, segmented, cv.CV_BGR2HSV)
        
        # Access threshold memory
        threshold = self._d_threshold
        temp_threshold = self._d_temp_threshold
        
        # Normalize image
        self._normalize(segmented, threshold)
        #cv.CvtColor(segmented, rotated, cv.CV_HSV2BGR)
        
        for algorithm in Settings.ALGORITHMS:
            
            if algorithm.enabled and algorithm.camera is Algorithm.Camera.DOWNWARD:
                
                for threshold_name in algorithm.thresholds:
                    
                    # Perform threshold to isolate a range of colors
                    threshold_values = algorithm.thresholds[threshold_name]
                    cv.InRangeS(segmented, threshold_values[0], threshold_values[1], threshold)
                    
                    self._reduce_noise(threshold)
                    
                    cv.Copy(threshold, temp_threshold)
                    point_sets = self._sample_points(temp_threshold, size, self._d_counter)
                    
                    self._publish_points(algorithm, point_sets, rotated, size, name=threshold_name)
        
        self._d_counter = (self._d_counter + 1) % Settings.SAMPLE_SIZE
        
        # Publish image
        try:
            self._downward_img_pub.publish(self._bridge.cv_to_imgmsg(rotated, "bgr8"))
        except rospy.ROSException:
            # Generally this exception occurs if ROS wasn't ready yet. We'll
            # just silently ignore it and next time should work
            pass
        except CvBridgeError, e:
            print e
    
    def _normalize(self, image, temp):
        """
        Runs linear stretch normalization filter on the Value
        channel of the given image. The given temp image is used to copy
        an individual channel in and out of to perform the actual normalization
        """
        
        #cv.SetImageCOI(image, 2) # Saturation
        #cv.Copy(image, temp)
        #cv.Normalize(temp, temp, 0, 255, cv.CV_MINMAX)
        #cv.Copy(temp, image)
        cv.SetImageCOI(image, 3) # Value
        cv.Copy(image, temp)
        cv.Normalize(temp, temp, 0, 255, cv.CV_MINMAX)
        cv.Copy(temp, image)
        cv.ResetImageROI(image)
    
    def _analyze_points(self, algorithm, points, image):
        """
        TODO
        """
        
        if algorithm.analysis == Algorithm.Analysis.RECTANGLE:
            
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
            
            return [(center, dims, rotation)]
            
        elif algorithm.analysis == Algorithm.Analysis.GATE:
            
            # Group all points into rows and columns
            rows = {}
            cols = {}
            for x, y in points:
                if x in cols:
                    cols[x].append(y)
                else:
                    cols[x] = [y]
                if y in rows:
                    rows[y].append(x)
                else:
                    rows[y] = [x]
            
            # Search for long rows
            lengths = [len(row) for row in rows.values()]
            avg_len = sum(lengths) / len(rows)
            max_len = max(lengths) * Settings.MAX_LENGTH_THRESHOLD
            if avg_len < max_len:
                long_rows = [y for y in rows.keys() if len(rows[y]) > max_len]
            else:
                long_rows = []
            long_rows.sort()
            
            # Search for long columns
            lengths = [len(col) for col in cols.values()]
            avg_len = sum(lengths) / len(cols)
            max_len = max(lengths) * Settings.MAX_LENGTH_THRESHOLD
            if avg_len < max_len:
                long_cols = [x for x in cols.keys() if len(cols[x]) > max_len]
            else:
                long_cols = []
            long_cols.sort()
            
            # Remove long column values from long rows
            for y in long_rows:
                rows[y] = [x for x in rows[y] if x not in long_cols]
            
            # Truncate row values from long columns based on max long row
            max_long_row = max(long_rows)
            for x in long_cols:
                cols[x] = [y for y in cols[x] if y <= max_long_row]
            
            rectangles = []
            
            # Create rectangle for horizontal section
            left = 1000
            right = 0
            for y in long_rows:
                left = min(left, min(rows[y]))
                right = max(right, max(rows[y]))
            min_long_row = min(long_rows)
            center = ((left + right) / 2, (max_long_row + min_long_row) / 2)
            dims = (right - left, max_long_row - min_long_row)
            rectangles.append((center, dims, 0))
            
            # Create rectangles for vertical sections
            # TODO: This is more tricky because we need to group the long_cols up first
            col_groups = []
            prev_x = None
            for x in long_cols:
                if prev_x is not None and x - prev_x == Settings.SAMPLE_SIZE: 
                    # Continuing a group
                    pass
                else:
                    # Starting a new group
                    if prev_x is not None:
                        col_groups.append(prev_x)
                    col_groups.append(x)
                prev_x = x
            if prev_x is not None:
                col_groups.append(prev_x)
            
            # Validate column groups
            if len(col_groups) % 2:
                # Oops it's odd... Truncate last item
                col_groups = col_groups[:-1]
            if len(col_groups) > 4:
                # Oops too many groups... Use heap to keep two largest groups
                heap = []
                for i in range(0, len(col_groups), 2):
                    heappush(heap, (col_groups[i+1] - col_groups[i], i))
                    if len(heap) > 2:
                        heappop(heap)
                new_groups = []
                for i in range(len(heap)):
                    group = heappop(heap)
                    new_groups.append(col_groups[group[1]])
                    new_groups.append(col_groups[group[1] + 1])
                col_groups = new_groups
            
            # Add rectangles from col_groups
            #TODO
            
            return rectangles
    
    def _publish_points(self, algorithm, point_sets, image, size, name):
        """
        Used by both image callbacks to publish details about the sets of
        points found. Also add annotations to the given image
        """
        
        # Limit number of sets to publish based on algorithm
        point_sets = nlargest(algorithm.max_point_sets, point_sets, len)
        
        # Find minimum area rotated rectangle around each set of points
        for points in point_sets:
            if points:
                
                for center, dims, rotation in self._analyze_points(algorithm, points, image):
                    
                    # Calculate confidence based on algorithm
                    if algorithm.confidence_type is Algorithm.Confidence.RECTANGLE:
                        expected_points = (dims[0] * dims[1]) / (Settings.SAMPLE_SIZE ** 2)
                    elif algorithm.confidence_type is Algorithm.Confidence.CIRCLE:
                        expected_points = (math.pi * dims[0] * dims[1]) / (4 * Settings.SAMPLE_SIZE ** 2)
                    confidence = min(len(points) / expected_points, 1)
                    
                    # Publish object data
                    self._get_publisher(algorithm).publish(ImgRecObject(
                        stamp = roslib.rostime.Time(time.time()),
                        name = name,
                        center_x = int(center[0])- size[0] / 2,
                        center_y = size[1] / 2 - int(center[1]),
                        rotation = rotation,
                        height = int(dims[0]),
                        width = int(dims[1]),
                        confidence = confidence
                    ))
    
    def _sample_points(self, image, size, offset):
        """
        Looks for all contiguous regions of white points in an image by sampling
        a shifting grid of points and performing an exhaustive 'adjacent points'
        search when a white point is found. Regions with a low number of points
        are excluded from the returned list
        """
        
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

