
from SubImageRecognition.msg import ImgRecObject

class Algorithm:
    """
    Defines all values needed by the ImageRecognition class in
    image_recognition.py to look for a certain amount of certain objects and
    publish data about them. Also defines constants used for a few of the
    values themselves and creates a publisher for the algorithm to be used later
    """
    
    DEFAULT = 'default'
    
    class Camera:
        FORWARD = 0
        DOWNWARD = 1
    
    class Analysis:
        RECTANGLE = 0
        GATE = 1
    
    class Confidence:
        RECTANGLE = 0
        CIRCLE = 1
    
    def __init__(self, enabled, name, camera, thresholds, analysis, max_point_sets, confidence_type):
        # Scale hue values down
        for threshold in thresholds.values():
            threshold[0][0] = int((threshold[0][0] * 179.0 / 255.0) + 0.5)
            threshold[1][0] = int((threshold[1][0] * 179.0 / 255.0) + 0.5)
        
        # Save arguments as class variables
        self.enabled = enabled
        self.name = name
        self.camera = camera
        self.thresholds = thresholds
        self.analysis = analysis
        self.max_point_sets = max_point_sets
        self.confidence_type = confidence_type
