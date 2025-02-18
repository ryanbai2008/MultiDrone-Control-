from djitellopy import Tello
import cv2
import time
import tello_tracking
import path_planner

#path
start_1_X, start_1_Y, end_1_X, end_1_Y = 120, 0, 0, -120
path1 = [start_1_X, start_1_Y, end_1_X, end_1_Y]

#drone current values
drone_1_pos = [path1[0], path1[1], 0] #(X, Y, angle), STARTING ANGLE MUST BE 0 DEGREES

#drone movement
drone_1_movement = [0, 0, 0] #(delta X, delta Y, delta angle)

#path planning and CV objects
drone_1_path_plan = path_planner.PathPlan(path1[0], path1[2], path1[1], path1[3], drone_1_pos[2])
drone_1_CV = tello_tracking.CV()

#goal reached for drones?
drone_1_terminate = False

#turn on drone
tello = Tello()
tello.connect()
tello.streamon()
tello.takeoff()

#timer for position updates
sleep_time = 0
timer = 0
iter = 0

#total time elapsed
start_time = time.time()
total_time = 0

##############################
##Drone initial orientation###
##############################
facing_human = False
print(tello.get_battery())
tello.send_rc_control(0, 0, 60, 0)
while not facing_human:
    #CV
    img1 = tello.get_frame_read().frame
    turn_1 = drone_1_CV.center_subject(img1, 1)

    #turn
    if turn_1 == 0: #if no human detected, continue turning
        turn_1 = 10
    elif turn_1 == 1: #human centered
        facing_human = True
        turn_1 = 0
    tello.send_rc_control(0, 0, 0, turn_1)
    time.sleep(0.1)

print("\n\n\nnow moving paths\n\n\n")
##############################
#########Path Planning########
##############################
drone_1_pos = tello.get_yaw()

while total_time < 15:
    #update total time
    total_time = time.time() - start_time

    #CV
    img1 = tello.get_frame_read().frame
    turn_1 = drone_1_CV.center_subject(img1, 1)
    if turn_1 == 1:
      turn_1 = 0
    
    if iter == 0:
        sleep_time = 0 #do not update positions for the first loop
        iter += 1
    else:
        sleep_time = time.time() - timer
    
    if not drone_1_terminate:
        print(f"updating position: {drone_1_pos}")
        drone_1_pos[0] += drone_1_movement[0] * sleep_time
        drone_1_pos[1] += drone_1_movement[1] * sleep_time
        drone_1_pos[2] = tello.get_yaw()
        # drone_1_pos[2] += turn_1 * sleep_time
        # drone_1_pos[2] = drone_1_pos[2] % 360
        print(f"updated  position: {drone_1_pos}")
    else:
        drone_1_pos[2] = tello.get_yaw()
        # drone_1_pos[2] += turn_1 * sleep_time
        # drone_1_pos[2] = drone_1_pos[2] % 360

    #path planning
    drone_1_movement = drone_1_path_plan.move_towards_goal(drone_1_pos[0], drone_1_pos[1], drone_1_pos[2], drone_1_terminate)
    if drone_1_movement[0] == 0.1:
        drone_1_terminate = True
        print("\n\n\nnow stopping drone movement\n\n\n")
        drone_1_movement[0], drone_1_movement[1] = 0, 0
    
    #move drone and update values, already considered if drone terminated
    tello.send_rc_control(drone_1_movement[0], drone_1_movement[1], 0, turn_1)

    timer = time.time() #time for keeping track of how much to update drones positions




#clean up
cv2.destroyAllWindows()
tello.land()
tello.streamoff()
tello.end()
