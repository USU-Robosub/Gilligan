from algorithm import Algorithm

class Settings:
    
    sample_size = 6
    min_point_set_len = 30
    root_topic = 'image_recognition/'
    
    algorithms = [
        
        # Forward Gate
        Algorithm(
            enabled = True,
            topic = root_topic + 'forward/gate',
            camera = Algorithm.FORWARD,
            thresholds = {
                Algorithm.DEFAULT: ((0, 0, 0), (143, 220, 70)),
            },
            max_point_sets = 2,
            confidence_type = Algorithm.RECTANGLE
        ),
        
        # Forward Buoys
        Algorithm(
            enabled = False,
            topic = root_topic + 'forward/buoys',
            camera = Algorithm.FORWARD,
            thresholds = {
                'red': ((120, 0, 0), (135, 150, 55)),
                'green': ((0, 150, 0), (125, 205, 55)),
                #'yellow': None, # TODO: Get good thresholds for yellow buoy
            },
            max_point_sets = 1,
            confidence_type = Algorithm.CIRCLE
        ),
        
        # TODO: Add more forward algorithms here
        
        # Downward Orange Rectangles
        Algorithm(
            enabled = True,
            topic = root_topic + 'downward/orange_rectangles',
            camera = Algorithm.DOWNWARD,
            thresholds = {
                'new': ((0, 0, 0), (143, 220, 70)),
                'old': ((5, 50, 50), (15, 255, 255)),
            },
            max_point_sets = 2,
            confidence_type = Algorithm.RECTANGLE
        ),
        
        # TODO: Add more downward algorithms here
        
    ]
