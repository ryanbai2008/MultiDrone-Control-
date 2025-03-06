from DJITelloPy import djitellopy
import cv2
import time
import tello_tracking
import path_planner
import avoid
import math

#path
start_1_X, start_1_Y, end_1_X, end_1_Y = 0, 0, 200, -200
path1 = [start_1_X, start_1_Y, end_1_X, end_1_Y]

#other drone path
path2 = [100, -150, 50, 150]

#drone current values
drone_1_pos = [path1[0], path1[1], 0] #(X, Y, angle), STARTING ANGLE MUST BE 0 DEGREES

#drone movement
drone_1_movement = [0, 0, 0] #(delta X, delta Y, delta angle)

#path planning and CV and collision objects
drone_1_path_plan = path_planner.PathPlan(path1[0], path1[2], path1[1], path1[3], drone_1_pos[2])
drone_1_CV = tello_tracking.CV()
drone_collision = avoid.Avoid(path1, path2)

#goal reached for drones?
drone_1_terminate = False

#turn on drone
tello = djitellopy.Tello()
tello.connect()
tello.streamon()
#tello.takeoff()
#tello.send_rc_control(0, 0, 60, 0)

#timer for position updates
sleep_time = 0
timer = 0
iter = 0

#drone heights
time.sleep(1)
drone_height = 200
normal_height = 200
# while tello.get_height() < normal_height:
#     tello.send_rc_control(0, 0, 0, 20)
# tello.send_rc_control(0, 0, 0, 0)
go_up = 0

#total time elapsed
start_time = time.time()
total_time = 0

##############################
##Drone initial orientation###
##############################
facing_human = False
print(f"battery: {tello.get_battery()}")
print(f"yaw: {tello.get_yaw()}")

while not facing_human:
    #CV
    img1 = tello.get_frame_read().frame
    turn_1 = drone_1_CV.center_subject(img1, 1)

    #turn
    if turn_1 == 0: #if no human detected, continue turning
        turn_1 = 25
    elif turn_1 == 1: #human centered
        facing_human = True
        turn_1 = 0
    #tello.send_rc_control(0, 0, 0, turn_1)
    time.sleep(0.3)

print("\n\n\nnow moving paths\n\n\n")
##############################
#########Path Planning########
##############################
drone_1_pos[2] = -1 * tello.get_yaw()

while total_time < 60:
    #update total time
    total_time = time.time() - start_time

    #CV
    img1 = tello.get_frame_read().frame
    turn_1 = drone_1_CV.center_subject(img1, 1)
    if turn_1 == 1:
      turn_1 = 0
    
    #update timer
    if iter == 0:
        sleep_time = 0 #do not update positions for the first loop
        iter += 1
    else:
        sleep_time = time.time() - timer
    
    if not drone_1_terminate:
        #calculate new angle
        print(f"updating position: {drone_1_pos}")
        drone_1_pos[2] = -1 * tello.get_yaw()

        #calculate new position
        theta_x_component_change = 0
        if drone_1_movement[0] > 0:
            theta_x_component_change = -1
        else:
            theta_x_component_change = 1
        theta_x_component = (drone_1_pos[2] + 90 * theta_x_component_change) % 360
        theta_y_component = (drone_1_pos[2]) % 360
        delta_x = abs(drone_1_movement[0]) * math.cos(math.radians(theta_x_component)) + drone_1_movement[1] * math.cos(math.radians(theta_y_component))
        delta_y = abs(drone_1_movement[0]) * math.sin(math.radians(theta_x_component)) + drone_1_movement[1] * math.sin(math.radians(theta_y_component))
        drone_1_pos[0] += delta_x * sleep_time
        drone_1_pos[1] += delta_y * sleep_time

        #detect collision and manage heights
        drone_height += go_up * sleep_time
        print(f"drone height: {drone_height}, normal height: {normal_height}")
        collision_check = drone_collision.detect_collision(drone_1_pos[0], drone_1_pos[1])
        if collision_check == "collision":
            go_up = 40
        elif collision_check == "no collision" and drone_height > normal_height * 1.1: #buffer
            go_up = -20
        elif collision_check == "no collision" and drone_height < normal_height * 1.1: #buffer
            go_up = 20
        else:
            go_up = 0
        
        if drone_height > 1.5 * normal_height:
            tello.land()
            break
        
        print(f"updated  position: {drone_1_pos}")
    else:
        go_up = 0
        drone_1_pos[2] = -1 * tello.get_yaw()

    #path planning
    drone_1_movement = drone_1_path_plan.move_towards_goal(drone_1_pos[0], drone_1_pos[1], drone_1_pos[2], drone_1_terminate)
    if drone_1_movement[0] == 0.1:
        drone_1_terminate = True
        print("\n\n\nnow stopping drone movement\n\n\n")
        drone_1_movement[0], drone_1_movement[1] = 0, 0
    
    #move drone and update values, already considered if drone terminated
    #tello.send_rc_control(drone_1_movement[0], drone_1_movement[1], go_up, turn_1)

    timer = time.time() #time for keeping track of how much to update drones positions




#clean up
time.sleep(5)
cv2.destroyAllWindows()
tello.send_rc_control(0, 0, 0, 0)
tello.land()
tello.streamoff()
tello.end()

