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
    
    ROOT_TOPIC = 'image_recognition/'
    
    ALGORITHMS = [
        
        # Forward Gate
        Algorithm(
            enabled = False,
            name = 'forward/gate',
            camera = Algorithm.Camera.FORWARD,
            thresholds = {
                Algorithm.DEFAULT: ((0, 0, 60), (250, 180, 135)),
            },
            analysis = Algorithm.Analysis.GATE,
            max_point_sets = 3,
            confidence_type = Algorithm.Confidence.RECTANGLE
        ),
        
        # Forward Buoys
        Algorithm(
            enabled = True,
            name = 'forward/buoys',
            camera = Algorithm.Camera.FORWARD,
            thresholds = {
                'red': ((135, 0, 30), (200, 210, 120)),
                'green': ((110, 200, 110), (130, 240, 200)),
                'yellow': ((95, 185, 160), (115, 240, 220))
            },
            analysis = Algorithm.Analysis.RECTANGLE,
            max_point_sets = 1,
            confidence_type = Algorithm.Confidence.CIRCLE
        ),
        
        # Forward Obstacle Course
        Algorithm(
            enabled = True,
            name = 'forward/obstacle_course',
            camera = Algorithm.Camera.FORWARD,
            thresholds = {
                Algorithm.DEFAULT: ((0, 0, 0), (255, 255, 255)),
            },
            analysis = Algorithm.Analysis.RECTANGLE,
            max_point_sets = 3,
            confidence_type = Algorithm.Confidence.RECTANGLE
        ),
        
        # TODO: Add more forward algorithms here
        
        # Downward Paths
        Algorithm(
            enabled = True,
            name = 'downward/paths',
            camera = Algorithm.Camera.DOWNWARD,
            thresholds = {
                Algorithm.DEFAULT: ((5, 50, 50), (15, 255, 255)),
            },
            analysis = Algorithm.Analysis.RECTANGLE,
            max_point_sets = 2,
            confidence_type = Algorithm.Confidence.RECTANGLE
        ),
        
        # TODO: Add more downward algorithms here
        
    ]

