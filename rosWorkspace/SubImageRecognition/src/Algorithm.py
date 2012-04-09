class Algorithm:
    
    enabled = False
    thresholds = {
        'default': ((0, 0, 0), (255, 255, 255)),
    }
    max_point_sets = 1
    confidence_type = 'rectangle'
    
    def __init__(self, enabled=None, thresholds=None, max_point_sets=None, confidence_type=None):
        if enabled is not None:
            self.enabled = enabled
        if thresholds is not None:
            self.thresholds = thresholds
        if max_point_sets is not None:
            self.max_point_sets = max_point_sets
        if confidence_type is not None:
            self.confidence_type = confidence_type
