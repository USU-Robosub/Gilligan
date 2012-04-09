import Algorithm

class Settings:
    
    sample_size = 6
    min_point_set_len = 30
    root_topic = 'image_recognition/'
    
    algorithms = {
        
        'forward_gate': Algorithm(
            enabled = True,
            thresholds = {
                'default': ((0, 0, 0), (143, 220, 70)),
            },
            max_point_sets = 2,
            confidence_type = 'circle'
        ),
        
        'forward_buoys': Algorithm(
            enabled = False,
            thresholds = {
                'red': None, # TODO: Get good threshold for red buoy
                'green': None, # TODO: Get good threshold for green buoy
                'yellow': None, # TODO: Get good threshold for yellow buoy
            },
            max_point_sets = 1,
            confidence_type = 'circle'
        ),
        
        # TODO: Add more forward algorithms here
        
        'downward_orange_rectangles': Algorithm(
            enabled = True,
            thresholds = {
                'default': ((5, 50, 50), (15, 255, 255)),
            },
            max_point_sets = 2,
            confidence_type = 'rectangle'
        ),
        
        # TODO: Add more downward algorithms here
        
    }
