import math

class Avoid:
    def __init__(self, path1, path2):
        self.start_1_X, self.start_1_Y, self.end_1_X, self.end_1_Y = path1
        self.start_2_X, self.start_2_Y, self.end_2_X, self.end_2_Y = path2
        self.slope_1 = (self.end_1_Y - self.start_1_Y) - (self.end_1_X - self.start_1_X)
        self.slope_2 = (self.end_2_Y - self.start_2_Y) - (self.end_2_X - self.start_2_X)
        self.magnitude_1 = math.sqrt((self.end_1_X - self.start_1_X) ** 2 + (self.end_1_Y - self.start_1_Y) ** 2)
        self.magnitude_2 = math.sqrt((self.end_2_X - self.start_2_X) ** 2 + (self.end_2_Y - self.start_2_Y) ** 2)

    def detect_collision(self, curr_X_1, curr_Y_1):
        percent_tot_travelled = math.sqrt((curr_X_1 - self.start_1_X) ** 2 + (curr_Y_1 - self.start_1_Y) ** 2) / self.magnitude_1
        curr_X_2 = self.start_2_X + percent_tot_travelled * (self.end_2_X - self.start_2_X)
        curr_Y_2 = self.start_2_Y + percent_tot_travelled * (self.end_2_Y - self.start_2_Y)
        distance = math.sqrt((curr_X_1 - curr_X_2) ** 2 + (curr_Y_1 - curr_Y_2) ** 2)
        if distance < 40:
            return "collision"
        else:
            return "no collision"
