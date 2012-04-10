import rospy

class Algorithm:
    
    DEFAULT = 'default'
    
    FORWARD = 0
    DOWNWARD = 1
    
    RECTANGLE = 0
    CIRCLE = 1
    
    enabled = False
    topic = None
    camera = FORWARD
    thresholds = {
        DEFAULT: ((0, 0, 0), (255, 255, 255)),
    }
    max_point_sets = 1
    confidence_type = RECTANGLE
    publisher = None
    
    def __init__(self, enabled=None, topic=None, camera=None, thresholds=None, max_point_sets=None, confidence_type=None):
        if enabled is not None:
            self.enabled = enabled
        if topic is not None:
            self.topic = topic
        if camera is not None:
            self.camera = camera
        if thresholds is not None:
            self.thresholds = thresholds
        if max_point_sets is not None:
            self.max_point_sets = max_point_sets
        if confidence_type is not None:
            self.confidence_type = confidence_type
        
        # Setup publisher for this algorithm
        if self.topic:
            self.publisher = rospy.Publisher(self.topic, ImgRecObject)
