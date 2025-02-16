import math


class pathPlan:
    def __init__(self, X_start, X_goal, Y_start, Y_goal, angle_start):
        self.X_start = X_start
        self.X_goal = X_goal
        self.Y_start = Y_start
        self.Y_goal = Y_goal
        self.angle_start = angle_start

        # angle_goal = omega - angle_start
        # self.omega = omega
        # self.X_goal, self.Y_goal = X_goal, Y_goal
        # self.diff_X_init, self.diff_Y_init, self.diff_angle_init = X_goal - X_start, Y_goal - Y_start, angle_goal - angle_start

    def move_towards_goal(self, X_curr, Y_curr, angle_curr):
        diff = [self.X_goal - X_curr, self.Y_goal - Y_curr]
        direction_vector = [math.cos(angle_curr), math.sin(angle_curr)]
        magnitude_diff = math.sqrt(diff[0] ** 2 + diff[1] ** 2)
        magnitude_direction_vector = math.sqrt(direction_vector[0] ** 2 + direction_vector[1] ** 2)

        angle_diff_smaller = math.acos((diff[0] * direction_vector[0] + diff[1] * direction_vector[1])/(magnitude_diff * magnitude_direction_vector))
        X_movement = math.sin(angle_diff_smaller) * magnitude_direction_vector
        Y_movement = math.cos(angle_diff_smaller) * magnitude_direction_vector
        # diff_X_curr, diff_Y_curr = self.X_goal - X_curr, self.Y_goal - Y_curr
        # if abs(diff_angle_curr / self.diff_angle_init) < 0.1: #if angle is within certain range of goal, terminate rotation
        #     if abs(diff_X_curr / self.diff_X_init) < 0.1 and abs(diff_Y_curr / self.diff_Y_init) < 0.1: #if (x,y) within certain range of goal state, terminate movement
        #         return {"terminate" : 1}
        #     else: #if goal state is not yet reached
        #         move_forward_val = 1
        #         if (angle_curr >= 270 or angle_curr <= 90):
        #             if (X_curr - self.X_goal > 0):
        #                 move_forward_val *= -1
        #             else:
        #                 pass
        #         else:
        #             if (X_curr - self.X_goal > 0):
        #                 pass
        #             else:
        #                 move_forward_val *= -1
        #         return {"move_left" : 0, "move_forward" : move_forward_val, "terminate" : 0}

        # else: #if angle goal is not yet reached
        #     move_left_val = 1
        #     if (self.omega > 0):
        #         move_left_val *= -1
        #     return {"move_left" : move_left_val, "move_forward" : 0, "terminate" : 0}