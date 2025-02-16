import pygame
from mapBackground import Background
import time
import threading

class localizer:
    def __init__(self, name, sizeCoeff, screen_height, screen_width, timeDur, rotationDur, angle, path, droneImg):
        self.name = name
        self.sizeCoeff = sizeCoeff
        self.angle = angle
        self.screen_height = screen_height
        self.screen_width = screen_width
        self.timeDur = timeDur
        self.rotationDur = rotationDur
        self.path = path
        self.droneImg = droneImg
        self.running = True


    def stop(self):
        self.running = False

    def drawPoints(self, screen, background, points, yaw):
        font = pygame.font.SysFont('Times',25)
        text = font.render('Straight Line' , True , (0, 0, 0)) 

        # Redraw the background image only at the previous position of the green dot
        #screen.blit(background.image, (0, 0))    
        
        for point in points:
            pygame.draw.circle(screen, (255, 0, 0), point, 3) #draws a red dot/line for the visited nodes/area
        # Rotate the image based on the yaw angle and draw it on the screen
        rotated_image = pygame.transform.rotate(self.droneImg, -yaw)
        image_rect = rotated_image.get_rect(center=(points[-1][0], points[-1][1]))
        screen.blit(rotated_image, image_rect.topleft)

        pygame.draw.circle(screen, (0, 255, 0), points[-1], 3)

        #adds positional text data, (0,0) is bottom left corner
        x_cord =(int)((self.screen_height+points[-1][0])/self.sizeCoeff)
        y_cord  = (int)((points[-1][1])/self.sizeCoeff)
        position_text = font.render(f'({x_cord}, {y_cord})cm', True, (255, 0, 0))
        screen.blit(position_text, (points[-1][0] + 10, points[-1][1] + 10))
        pygame.display.update()


    def localizeIRT(self, screen, backgroundimg):   
        background = Background(backgroundimg, [0, 0], 1)
        positions_lock = threading.Lock()
        x,y = 0, 0 # origin

        points = [(x, y)]
        updateTime = 0.04 #amount of tims we want to step (CHANGE THIS VALUE DEPENDING ON ACCCURACY, Controls odometry accuracy)
        angleUpdateTime = 0.005
        step = 0
        yaw = 0

        start_pos = self.path[0]
        end_pos = self.path[1]

        # Initialize the current position
        current_pos = list(start_pos)
        previous_pos = current_pos.copy()
        num_steps = int(self.timeDur / updateTime)
        angle_num_steps = int(self.rotationDur / angleUpdateTime)

        # Calculate the increments in x and y directions
        dx = (end_pos[0] - start_pos[0]) / num_steps
        dy = (end_pos[1] - start_pos[1]) / num_steps

        initial_yaw = 0  # Initial yaw angle in degrees
        target_yaw = self.angle  # Target yaw angle in degrees (can be adjusted)

        points.append(current_pos)

        #start_time = pygame.time.get_ticks()  # Get the start time
        #rotation_done = False
        self.drawPoints(screen, background, points, yaw)
        #delays for the takeoff time
        time.sleep(3)
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False    
        
            if yaw != target_yaw:
                yaw += (target_yaw - initial_yaw) / angle_num_steps
                if abs(yaw - target_yaw) < 0.1:
                    yaw = target_yaw  # Snap to target yaw if close
            else:
                if step <= num_steps:
                    previous_pos = current_pos.copy()
                    current_pos[0] += dx
                    current_pos[1] += dy
                    points.append(tuple(current_pos))
                    # Increment yaw angle
                    points.append((int(current_pos[0]), int(current_pos[1])))
                    step += 1
            # Draw the frame
            self.drawPoints(screen, background, points, yaw)

            pygame.time.delay(int(updateTime))


        # Ensure the final position is exactly the end position
        current_pos = end_pos
        points.append(tuple(current_pos))

    
    def get_position_and_rotation(self):
        return self.position, self.rotation_angle