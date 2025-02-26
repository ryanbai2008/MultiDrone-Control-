import math

class PathPlan:
    def __init__(self, X_start, X_goal, Y_start, Y_goal, angle_start):
        self.X_start = X_start
        self.X_goal = X_goal
        self.Y_start = Y_start
        self.Y_goal = Y_goal
        self.angle_start = angle_start
        self.original_magnitude_diff = math.sqrt((X_goal - X_start) ** 2 + (Y_goal - Y_start) ** 2)

    def move_towards_goal(self, X_curr, Y_curr, angle_curr, terminate):
        #skip if drone has already reached position
        if terminate:
            return [0, 0]
        
        #vectors
        diff = [self.X_goal - X_curr, self.Y_goal - Y_curr]
        direction_vector = [math.cos(math.radians(angle_curr)), math.sin(math.radians(angle_curr))]

        #magnitudes
        magnitude_diff = math.sqrt(diff[0] ** 2 + diff[1] ** 2)
        magnitude_direction_vector = math.sqrt(direction_vector[0] ** 2 + direction_vector[1] ** 2)

        #goal reached if within certain range of goal
        if magnitude_diff / self.original_magnitude_diff < 0.05:
            return [0.1, 0.1] 
        
        #if goal not reached, calculate movement neccesary to move one small step closer to goal
        angle_diff_smaller = math.acos((diff[0] * direction_vector[0] + diff[1] * direction_vector[1])/(magnitude_diff * magnitude_direction_vector)) #u.v/|u||v| = cos(theta), theta < 180
        
        movement_scale = 0.1 #change for smoother or less smooth path
        #sign of Y_movement is always correct, but X_movement is not, change of basis to determine
        X_movement = math.sin(angle_diff_smaller) * self.original_magnitude_diff * movement_scale 
        Y_movement = math.cos(angle_diff_smaller) * self.original_magnitude_diff * movement_scale 

        #change of basis, row reduction to solve, done on paper & simplified on code for efficiency:
        x, y = direction_vector
        a, b = diff
        n = (a * y - b * x) / (magnitude_direction_vector ** 2) #scalar for rightwards basis vector
        if n < 0:
            X_movement *= -1

        return [int(X_movement), int(Y_movement)] #right, forward
