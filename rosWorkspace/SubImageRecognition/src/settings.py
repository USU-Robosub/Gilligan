from algorithm import Algorithm

class Settings:
    """
    Container class for storing all image recognition settings. Both Algorithm
    in algorithm.py and ImageRecognition in image_recognition.py refer to this
    class for common settings
    """
    
    SAMPLE_SIZE = 6
    MIN_POINTS = 30
    
    MAX_LENGTH_THRESHOLD = 0.8
    
    TOPIC_ROOT = 'image_recognition/'
    TOPIC_FORWARD = ''
    TOPIC_DOWNWARD = ''
    
    ALGORITHMS = [
        
        # Forward Gate
        Algorithm(
            enabled = False,
            name = 'forward/gate',
            camera = Algorithm.Camera.FORWARD,
            thresholds = ([0, 0, 0], [250, 180, 60]), # ([0, 0, 60], [250, 180, 135]), # Old values, when we were looking for 2 orange and 1 black rectangle
            analysis = Algorithm.Analysis.RECTANGLE,
            max_point_sets = 12,
            confidence_type = Algorithm.Confidence.RECTANGLE,
            annotation_color = (0, 128, 255), # Orange
            annotation_type = Algorithm.Annotation.ROTATION
        ),
        
        # Forward Buoys
        Algorithm(
            enabled = True,
            name = 'buoys/red',
            camera = Algorithm.Camera.FORWARD,
            thresholds = ([135, 0, 30], [200, 210, 120]),
            analysis = Algorithm.Analysis.RECTANGLE,
            max_point_sets = 1,
            confidence_type = Algorithm.Confidence.CIRCLE,
            annotation_color = (0, 0, 255), # Red
            annotation_type = Algorithm.Annotation.RADIUS
        ),
        Algorithm(
            enabled = False,
            name = 'buoys/green',
            camera = Algorithm.Camera.FORWARD,
            thresholds = ([110, 200, 110], [130, 240, 200]),
            analysis = Algorithm.Analysis.RECTANGLE,
            max_point_sets = 1,
            confidence_type = Algorithm.Confidence.CIRCLE,
            annotation_color = (0, 255, 0), # Green
            annotation_type = Algorithm.Annotation.RADIUS
        ),
        Algorithm(
            enabled = True,
            name = 'buoys/yellow',
            camera = Algorithm.Camera.FORWARD,
            thresholds = ([95, 185, 160], [115, 240, 220]),
            analysis = Algorithm.Analysis.RECTANGLE,
            max_point_sets = 1,
            confidence_type = Algorithm.Confidence.CIRCLE,
            annotation_color = (0, 255, 255), # Yellow
            annotation_type = Algorithm.Annotation.RADIUS
        ),
        
        # Forward Obstacle Course
        Algorithm(
            enabled = True,
            name = 'obstacle_course',
            camera = Algorithm.Camera.FORWARD,
            thresholds = ([0, 0, 0], [255, 255, 255]), # TODO: Get real thresholds for this
            analysis = Algorithm.Analysis.RECTANGLE,
            max_point_sets = 3,
            confidence_type = Algorithm.Confidence.RECTANGLE,
            annotation_color = (255, 0, 0), # Blue
            annotation_type = Algorithm.Annotation.ROTATION
        ),
        
        # Downward Paths
        Algorithm(
            enabled = True,
            name = 'paths',
            camera = Algorithm.Camera.DOWNWARD,
            thresholds = ([5, 50, 50], [15, 255, 255]),
            analysis = Algorithm.Analysis.RECTANGLE,
            max_point_sets = 2,
            confidence_type = Algorithm.Confidence.RECTANGLE,
            annotation_color = (0, 128, 255), # Orange
            annotation_type = Algorithm.Annotation.ROTATION
        ),
        
    ]
