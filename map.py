
import math
from mapBackground import Background
import pygame
#Used to create a map

class mapStart:
    def __init__(self, sizeCoeff, screen, backgroundimg):
        self.sizeCoeff = sizeCoeff
        self.screen = screen
        self.backgroundimg = backgroundimg
        self.angle = 0
  
    def getEdgeWeight(self, pos0, pos1):
        #gets the edge weights between nodes
        x1 = abs(pos0[0]-pos1[0])
        y1 = abs(pos0[1]-pos1[1])#euclidean distance
        dist_px = math.hypot(x1, y1)
        dist_cm = dist_px * self.sizeCoeff
        return int(dist_cm), int(dist_px)

    def get_angle(self, pos0, pos1, posref):
        #gets angle between 2 lines
        ax =posref[0] - pos0[0]
        ay =posref[1] - pos0[1]
        bx =posref[0] - pos1[0]
        by =posref[1] - pos1[1]
        dot = (ax*bx) + (ay*by) #dot product
        magA = math.sqrt(ax**2 + ay**2)
        magB = math.sqrt(bx**2 + by**2)

        if magA == 0 or magB == 0:
                    return 0 

        rad = math.acos(dot/(magA*magB))
        self.angle = (rad * 360)/(2*math.pi) #convert to degree

        # Calculate cross product
        cross = (ax * by) - (ay * bx)
        
        # Determine the sign of the angle
        if cross < 0:
            self.angle = -self.angle

        return (int)(self.angle) 
    
    def getRadius(self, pos0, pos1):
        #gets radius of line
        x1 = abs(pos0[0]-pos1[0])
        y1 = abs(pos0[1]-pos1[1])
        radius = math.hypot(x1, y1)
        return radius
    
    def createMap(self): 
        isRunning = True
        path = [] #used for making a path between 2 points
        index= 0 #to keep track of if there is more than 1 line (updates the mouse position)

        bG = self.backgroundimg
        self.screen.blit(bG.image, bG.rect)
        #pygame.draw.rect(screen, (255, 255, 255), rect=(0, 500, 200, 200))

        while isRunning:
            #if the user quits, stops the loop
            pos = pygame.mouse.get_pos() #gets the mouse position

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    isRunning = False
            
                elif event.type == pygame.MOUSEBUTTONDOWN:        
                    pos = pygame.mouse.get_pos()      
                    posx, posy = pygame.mouse.get_pos() #gets the mouse position
                    if posy >105:
                        path.append(pos)
                        if index>0:
                            #pygame.draw.line(self.screen, (0, 0, 0), path[index-1], pos, 6) #creates a line as the edges                
                            isRunning = False
                        pygame.draw.circle(self.screen, (0, 0, 255), pos, 5) #To note where the nodes are
                        index +=1
            pygame.display.flip()
            
        #computer distances and angles
        pathAngle = []
        distanceInCm = []
        distanceInPx = []
        #first pos ref
        path.insert(0, (path[0][0], path[0][1] - 10))

        #calculates the distances and angles
        for index in range(len(path)):
            if index >1:
                dist_cm, dist_px = self.getEdgeWeight(path[index-1], path[index])
                distanceInCm.append(dist_cm)
                distanceInPx.append(dist_px)
            if index>0 and index<(len(path)-1):
                angle = self.get_angle(path[index-1], path[index+1], path[index])
                pathAngle.append(angle)

        return self.angle, dist_cm, dist_px, path
    
    def addPerson(self, sizeCoeff):
        running = True
        personpos = []
        screen_width, screen_height = pygame.display.get_surface().get_size()
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
        
                elif event.type == pygame.MOUSEBUTTONDOWN:        
                    pos = pygame.mouse.get_pos()      
                    posx, posy = pygame.mouse.get_pos() #gets the mouse position
                    if posy >105:
                        personpos.append(pos)
                        pygame.draw.circle(self.screen, (255, 255, 0), pos, 15) #To note where the person is 
                        font = pygame.font.SysFont('Times',25)
                        intersectx = (int)((posx)*sizeCoeff)
                        intersecty = (int)((screen_height  - posy)*sizeCoeff)
                        position_text = font.render(f'({intersectx}, {intersecty})cm', True, (255, 255, 0))
                        self.screen.blit(position_text, (posx+10, posy+10))
                        running = False
        return intersectx, intersecty, pos
    
    def calculatePoint(self, radius, point1, point2):
        #using both radii, gets distance between each
        #get the equation of the line
        x1, y1 = point1
        x2, y2 = point2

        if x2 - x1 != 0:
            slope = (y1 - y2) / (x1 - x2)
        else:
            slope = float('inf')  # handle vertical lines

        # Normalize the direction vector
        if slope != float('inf'):
            direction_vector = (1, slope)
        else:
            direction_vector = (0, 1)
        magnitude = math.sqrt(1 + slope ** 2)
        unit_vector = (direction_vector[0] / magnitude, direction_vector[1] / magnitude)

        new_x = x2  + radius * (unit_vector[0])
        new_y = y2  + radius * (unit_vector[1])
        if((new_x < x2 and x1>x2) or (new_y<y2 and y1>y2)):
            new_x = -new_x
            new_y = -new_y

        if((new_x > x2 and x1<x2) or (new_y>y2 and y1<y2)):
            new_x = -new_x
            new_y = -new_y

        print(f"angle: {new_x}")
        print(f"angle: {new_y}")

        if(new_x>864 or new_x<0 or new_y>585 or new_y<150):
            return "fail"
        return (new_x, new_y)
        
    
    def add_circular_pattern(self, path, personpos):
        #pygame.draw.line(self.screen, (211, 211, 211, 45), path[1], personpos, 6) #creates a line to show                
        #pygame.draw.line(self.screen, (255, 255, 255, 45), path[2], personpos, 6) #creates a line                 
        radius = self.getRadius(personpos, path[1]) #gets the radius from the person to the first point
        radius2 = self.getRadius(personpos, path[2]) #gets the radius from the person to the second point
        angle = self.get_angle(path[1], path[2], personpos)
        #ADD IT SO THAT IT DRAWS A POINT ON THE LINE OF PATH 2
        pointCords = self.calculatePoint(radius, path[2], personpos)
        if(pointCords == "fail"):
            print("Failed")
        # Calculate the angle of each point using atan2
        angle2 = (math.atan2(path[1][1] - personpos[1], path[1][0] - personpos[0]))
        angle1 = (math.atan2(pointCords[1] - personpos[1], pointCords[0] - personpos[0]))
            
        if angle1 > angle2:
            if angle1 - angle2 > math.pi:
                angle1 -= 2 * math.pi
        else:
            if angle2 - angle1 > math.pi:
                angle2 -= 2 * math.pi

        #swaps angles if they are less
        if angle1 < angle2:
           temp = angle2
           angle2 = angle1
           angle1 = temp

        pygame.draw.arc(self.screen, (0,0,0), (personpos[0] - radius, personpos[1]-radius, 2 * radius, 2 * radius), -angle1, -angle2+0.01, 4)
        pygame.draw.line(self.screen, (0,0,0), path[2], pointCords, 6)
        pygame.draw.circle(self.screen, (0, 0, 255), pointCords, 5) #To note where the nodes are
        pygame.draw.circle(self.screen, (0, 0, 255), path[1], 5) #To note where the nodes are
        pygame.draw.circle(self.screen, (0, 0, 255), path[2], 5) #To note where the nodes are

        return radius, angle


class initializeMap:
    def __init__(self, screen, instruction):
        self.screen = screen
        self.angle = 0
        self.instruction = instruction


    def scaleImgDown(self, img, scale_factor):
        original_width, original_height = img.get_size()
        new_width = int(original_width * scale_factor)
        new_height = int(original_height * scale_factor)
        img = pygame.transform.smoothscale(img, (new_width, new_height))# Scale the image
        return img
    
    def changeInstruction(self, instruction): 
        #Changes the instruction 
        screen_width, screen_height = pygame.display.get_surface().get_size()
        rect = pygame.Rect(0, 0, screen_width, 105)
        pygame.draw.rect(self.screen, (255, 255, 255), rect)
        pygame.display.update(rect)
        self.instruction = instruction

    def start_screen(self, batterylvl1, speedx1, speedz1, altitude1, batterylvl2, speedx2, speedz2, altitude2):
        #Initializes the screen for mapping
        screen_width, screen_height = pygame.display.get_surface().get_size()
        pygame.draw.line(self.screen, (0, 0, 0), (0, 100), (screen_width, 100), 20)

        instructionfont = pygame.font.SysFont('Comic Sans', 40)
        instructtext = instructionfont.render(self.instruction, True , (0, 0, 10)) 
        self.screen.blit(instructtext, ((screen_width/2)-(instructtext.get_width()/2), 25)) #Gives instructions based on what is going on

        tello1logo = pygame.image.load("tello1logo.png")  
        tello1logo = self.scaleImgDown(tello1logo, 0.09)
        self.screen.blit(tello1logo, (0, 0))  

        batterylow = pygame.image.load("batteries/onebar.png") 
        batterylow = self.scaleImgDown(batterylow, 0.075)
        batterymedium = pygame.image.load("batteries/twobar.png") 
        batterymedium = self.scaleImgDown(batterymedium, 0.075)
        batteryhigh = pygame.image.load("batteries/threebar.png")
        batteryhigh = self.scaleImgDown(batteryhigh, 0.075)
        batterydead = pygame.image.load("batteries/dead.png") 
        batterydead = self.scaleImgDown(batterydead, 0.075)
        batteryfull = pygame.image.load("batteries/full.png") 
        batteryfull = self.scaleImgDown(batteryfull, 0.075)

        percent = str(batterylvl1) + "%"
        batteryfont = pygame.font.SysFont('Comic Sans', 12)
        batterytext = batteryfont.render(percent, True , (0, 0, 10)) 
        self.screen.blit(batterytext, (70, 18))

        xSpeedText = "Linear Speed:" + str(speedx1) + " cm/s"
        speedFont = pygame.font.SysFont('Comic Sans', 10)
        xSpeedText = speedFont.render(xSpeedText, True , (0, 0, 10)) 
        self.screen.blit(xSpeedText, (75, 34))

        zSpeedText = "Angular Speed:" + str(speedz1) + " deg/s"
        zSpeedText = speedFont.render(zSpeedText, True , (0, 0, 10)) 
        self.screen.blit(zSpeedText, (75, 50))

        altitudeText = "Height: " + str(altitude1) + " cm"
        altitudeText = speedFont.render(altitudeText, True , (0, 0, 10)) 
        self.screen.blit(altitudeText, (70, 66))
        
        if batterylvl1 is not None:
            if(batterylvl1 <= 5):
                self.screen.blit(batterydead, (80, 0))  
            elif(batterylvl1 <= 25):
                self.screen.blit(batterylow, (80, 0))  
            elif(batterylvl1<50):
                self.screen.blit(batterymedium, (80, 0))  
            elif(batterylvl1<80):
                self.screen.blit(batteryhigh, (80, 0))  
            else:
                self.screen.blit(batteryfull, (80, 0))  

        #SECOND DRONE
        tello2logo = pygame.image.load("tello2logo.png")  # Replace with the path to your image file
        tello2logo = self.scaleImgDown(tello2logo, 0.09)
        self.screen.blit(tello2logo, (screen_width-tello1logo.get_width(), 0)) 

        percent2 = str(batterylvl2) + "%"
        batterytext2 = batteryfont.render(percent2, True , (0, 0, 10)) 
        self.screen.blit(batterytext2, (screen_width-120, 18))

        xSpeedText2 = "Linear Speed:" + str(speedx2) + " cm/s"
        speedFont = pygame.font.SysFont('Comic Sans', 10)
        xSpeedText2 = speedFont.render(xSpeedText2, True , (0, 0, 10)) 
        self.screen.blit(xSpeedText2, (screen_width-178, 34))

        zSpeedText2 = "Angular Speed:" + str(speedz2) + " deg/s"
        zSpeedText2 = speedFont.render(zSpeedText2, True , (0, 0, 10)) 
        self.screen.blit(zSpeedText2, (screen_width-178, 50))

        altitudeText2 = "Height: " + str(altitude2) + " cm"
        altitudeText2 = speedFont.render(altitudeText2, True , (0, 0, 10)) 
        self.screen.blit(altitudeText2, (screen_width-145, 66))

        if batterylvl2 is not None:
            if(batterylvl2 <= 5):
                self.screen.blit(batterydead, (screen_width-110, 0))  
            elif(batterylvl2 <= 25):
                self.screen.blit(batterylow, (screen_width-110, 0))  
            elif(batterylvl2<50):
                self.screen.blit(batterymedium, (screen_width-110, 0))  
            elif(batterylvl2<80):
                self.screen.blit(batteryhigh, (screen_width-110, 0))  
            else:
                self.screen.blit(batteryfull, (screen_width-110, 0))  
