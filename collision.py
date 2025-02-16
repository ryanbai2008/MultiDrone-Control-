import math
import customtello

class avoider:
    def __init__(self, pos, intersection, speed1, speed2):
        self.intersection = intersection
        self.pos = pos
        self.speed = speed1 #speed of the second drone which will move up
        self.speed2 = speed2 #speed of first drone

    def get_vertex(self):
        return None

    # Calculate parabolic path
    def calculate_parabolic_path_3d(intersection_point, height, initial_position, ascend_distance):
        h, k = intersection_point
        z_intersect = height
        x_initial, y_initial = initial_position
        z_initial = height
        x_ascend_start = h - ascend_distance
        a = (y_initial - k) / (x_initial - h)**2
        b = (z_initial - z_intersect) / (x_initial - h)**2
        
        def parabolic_path(x):
            if x < x_ascend_start:
                return y_initial, z_initial
            else:
                y_value = a * (x - h)**2 + k
                z_value = b * (x - h)**2 + z_intersect
                return y_value, z_value
        
        return parabolic_path

