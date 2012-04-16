import rospy
from settings import Settings
from SubImageRecognition.msg import ImgRecObject

class Algorithm:
    """
    Defines all values needed by the ImageRecognition class in
    image_recognition.py to look for a certain amount of certain objects and
    publish data about them. Also defines constants used for a few of the
    values themselves and creates a publisher for the algorithm to be used later
    """
    
    DEFAULT = 'default'
    
    FORWARD = 0
    DOWNWARD = 1
    
    RECTANGLE = 0
    CIRCLE = 1
    
    def __init__(self, enabled, name, camera, thresholds, max_point_sets, confidence_type):
        # Save arguments as class variables
        self.enabled = enabled
        self.name = name
        self.camera = camera
        self.thresholds = thresholds
        self.max_point_sets = max_point_sets
        self.confidence_type = confidence_type
        
        # Setup publisher for this algorithm
        if self.name:
            self.publisher = rospy.Publisher(Settings.ROOT_TOPIC + self.name, ImgRecObject)
        else:
            self.publisher = None
