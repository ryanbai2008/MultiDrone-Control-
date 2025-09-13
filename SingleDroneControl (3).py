import cv2
import time
import tello_tracking
import path_planner
# import avoid
import math
from djitellopy import Tello

# ------------------------------
# PATH DEFINITIONS
# ------------------------------
start_1_X, start_1_Y, end_1_X, end_1_Y = 0, 0, 300, 300
path1 = [start_1_X, start_1_Y, end_1_X, end_1_Y]

# # Other drone path
# path2 = [0, 0, -500, -500]

# Drone current values (X, Y, angle). STARTING ANGLE MUST BE 0 DEGREES
drone_1_pos = [path1[0], path1[1], 0]

# Drone movement (delta X, delta Y, delta angle)
drone_1_movement = [0, 0, 0]

# Path planning, CV, and collision objects
drone_1_path_plan = path_planner.PathPlan(path1[0], path1[2], path1[1], path1[3], drone_1_pos[2])
drone_1_CV = tello_tracking.CV()
# drone_collision = avoid.Avoid(path1, path2)

# Goal reached?
drone_1_terminate = False

# ------------------------------
# TELLO SETUP
# ------------------------------
tello = Tello()
tello.connect()
tello.streamon()
tello.takeoff()
frame_read = tello.get_frame_read()
while frame_read.frame is None or frame_read.frame.size == 0:
    time.sleep(0.1)

# Get frame size
frame = frame_read.frame
use_half_res = False   # set False if you want full size

if use_half_res:
    frame = cv2.resize(frame, (frame.shape[1] // 2, frame.shape[0] // 2))

frame_height, frame_width = frame.shape[:2]

fourcc = cv2.VideoWriter_fourcc(*'MJPG')  # try 'XVID' or 'MJPG' if this fails
out = cv2.VideoWriter("output.avi", fourcc, 30, (frame_width, frame_height))
upwards_velo = 50
tello.send_rc_control(0, 0, upwards_velo, 0)

# Timer for position updates
sleep_time = 0
timer = 0
iter = 0

# Drone heights
time.sleep(2)
tello.send_rc_control(0, 0, 0, 0)
drone_height = upwards_velo * 2
normal_height = upwards_velo * 2
go_up = 0

# Total time elapsed
start_time = time.time()
total_time = 0

# ------------------------------
# INITIAL ORIENTATION
# ------------------------------
try:
    facing_human = False
    print(f"battery: {tello.get_battery()}")

    while not facing_human:
        frame = frame_read.frame
        if frame is None or frame.size == 0:
            continue  # skip invalid frames
        turn_1, detections = drone_1_CV.center_subject(frame)
        for det in detections:
            x, y, w, h = det["box"]
            label = f"{det['label']} {det['confidence']:.2f}"
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2) #comment out if doesnt work
            cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 0), 2)

        # Write frame
        frame = cv2.resize(frame, (frame_width, frame_height))
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        success = out.write(frame)
        # If no human detected, keep turning
        if turn_1 == 0:
            turn_1 = 25
        elif turn_1 == 1:  # Human centered
            facing_human = True
            turn_1 = 0

        # Send turn command (commented for safety while testing)
        tello.send_rc_control(0, 0, 0, turn_1)

    print("\n\n\nnow moving paths\n\n\n")

    # ------------------------------
    # PATH PLANNING LOOP
    # ------------------------------
    drone_1_pos[2] = -1 * tello.get_yaw()

    while total_time < 60:
        total_time = time.time() - start_time
        frame = frame_read.frame
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        if frame is None or frame.size == 0:
            continue  # skip invalid frames
        turn_1, detections = drone_1_CV.center_subject(frame)
        for det in detections:
            x, y, w, h = det["box"]
            label = f"{det['label']} {det['confidence']:.2f}"
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2) #comment out if doesnt work
            cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 0), 2)

        # Write frame
        frame = cv2.resize(frame, (frame_width, frame_height))
        success = out.write(frame)
        if not success:
            print("[WARN] Failed to write frame")

        if turn_1 == 1:  # Human centered
            turn_1 = 0

        # Update timer
        if iter == 0:
            sleep_time = 0
            iter += 1
        else:
            sleep_time = time.time() - timer

        if not drone_1_terminate:
            # Calculate new angle
            print(f"updating position: {drone_1_pos}")
            drone_1_pos[2] = -1 * tello.get_yaw()

            # Calculate new position
            theta_x_component_change = -1 if drone_1_movement[0] > 0 else 1
            theta_x_component = (drone_1_pos[2] + 90 * theta_x_component_change) % 360
            theta_y_component = (drone_1_pos[2]) % 360

            delta_x = (abs(drone_1_movement[0]) * math.cos(math.radians(theta_x_component))
                       + drone_1_movement[1] * math.cos(math.radians(theta_y_component)))
            delta_y = (abs(drone_1_movement[0]) * math.sin(math.radians(theta_x_component))
                       + drone_1_movement[1] * math.sin(math.radians(theta_y_component)))

            drone_1_pos[0] += delta_x * sleep_time
            drone_1_pos[1] += delta_y * sleep_time

            # Collision detection and height control
            # drone_height += go_up * sleep_time
            print(f"drone height: {drone_height}, normal height: {normal_height}")

            # collision_check = drone_collision.detect_collision(drone_1_pos[0], drone_1_pos[1])
            # if collision_check == "collision":
            #     go_up = 40
            # elif collision_check == "no collision" and drone_height > normal_height * 1.1:
            #     go_up = -20
            # elif collision_check == "no collision" and drone_height < normal_height * 1.1:
            #     go_up = 20
            # else:
            #     go_up = 0
            go_up = 0

            if drone_height > 1.5 * normal_height:
                tello.land()
                break

            print(f"updated position: {drone_1_pos}")

        else:
            go_up = 0
            drone_1_pos[2] = -1 * tello.get_yaw()

        # Path planning
        drone_1_movement = drone_1_path_plan.move_towards_goal(
            drone_1_pos[0], drone_1_pos[1], drone_1_pos[2], drone_1_terminate
        )

        if drone_1_movement[0] == 0.1:
            drone_1_terminate = True
            print("\n\n\nnow stopping drone movement\n\n\n")
            drone_1_movement[0], drone_1_movement[1] = 0, 0

        # Move drone
        go_up = 0
        tello.send_rc_control(drone_1_movement[0], drone_1_movement[1], go_up, turn_1)

        # Update timer
        timer = time.time()

except KeyboardInterrupt:
    print("\nManual stop requested. Landing drone...")

finally:
    # Clean up
    out.release()
    time.sleep(0.3)
    cv2.destroyAllWindows()
    tello.send_rc_control(0, 0, 0, 0)
    tello.land()
    tello.streamoff()
    tello.end()






# import cv2
# import time
# import tello_tracking
# from djitellopy import Tello

# # Init Tello
# tello = Tello()
# tello.connect()
# tello.streamon()

# # Background frame reader
# frame_read = tello.get_frame_read()

# # Wait for a valid frame
# while frame_read.frame is None or frame_read.frame.size == 0:
#     time.sleep(0.1)

# # Get frame size
# frame = frame_read.frame
# use_half_res = False   # set False if you want full size

# if use_half_res:
#     frame = cv2.resize(frame, (frame.shape[1] // 2, frame.shape[0] // 2))

# frame_height, frame_width = frame.shape[:2]
# print(f"Recording video at resolution: {frame_width}x{frame_height}")

# # Initialize VideoWriter with matching size
# fourcc = cv2.VideoWriter_fourcc(*'MJPG')  # try 'XVID' or 'MJPG' if this fails
# out = cv2.VideoWriter("output.avi", fourcc, 45, (frame_width, frame_height))

# # Your CV object
# DRONE_CV_OBJ = tello_tracking.CV()

# start_time = time.time()
# while time.time() - start_time < 60:
#     frame = frame_read.frame
#     if frame is None or frame.size == 0:
#         continue  # skip invalid frames

#     turn, detections = DRONE_CV_OBJ.center_subject(frame)

#     # Draw detections
#     for det in detections:
#         x, y, w, h = det["box"]
#         label = f"{det['label']} {det['confidence']:.2f}"
#         cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2) #comment out if doesnt work
#         cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
#                     0.6, (0, 255, 0), 2)

#     # Write frame
#     frame = cv2.resize(frame, (frame_width, frame_height))
#     success = out.write(frame)
#     if not success:
#         print("[WARN] Failed to write frame")

# # Cleanup
# out.release()
# tello.streamoff()
# tello.end()









# import cv2
# import time
# import tello_tracking
# import path_planner
# import avoid
# import math
# from djitellopy import Tello
# # from DJITelloPy import djitellopy

# #path
# start_1_X, start_1_Y, end_1_X, end_1_Y = 0, 0, 0, 10
# path1 = [start_1_X, start_1_Y, end_1_X, end_1_Y]

# #other drone path
# path2 = [-100, -150, -50, 150]

# #drone current values
# drone_1_pos = [path1[0], path1[1], 0] #(X, Y, angle), STARTING ANGLE MUST BE 0 DEGREES

# #drone movement
# drone_1_movement = [0, 0, 0] #(delta X, delta Y, delta angle)

# #path planning and CV and collision objects
# drone_1_path_plan = path_planner.PathPlan(path1[0], path1[2], path1[1], path1[3], drone_1_pos[2])
# drone_1_CV = tello_tracking.CV()
# drone_collision = avoid.Avoid(path1, path2)

# #goal reached for drones?
# drone_1_terminate = False

# #turn on drone
# tello = Tello()
# tello.connect()
# tello.streamon()
# tello.takeoff()
# upwards_velo = 25
# tello.send_rc_control(0, 0, upwards_velo, 0)

# #timer for position updates
# sleep_time = 0
# timer = 0
# iter = 0

# #drone heights
# time.sleep(1)
# tello.send_rc_control(0, 0, 0, 0)
# drone_height = upwards_velo
# normal_height = upwards_velo
# go_up = 0

# #total time elapsed
# start_time = time.time()
# total_time = 0

# try:
#     ##############################
#     ##Drone initial orientation###
#     ##############################
#     facing_human = False
#     print(f"battery: {tello.get_battery()}")

#     while not facing_human:
#         #CV
#         img1 = tello.get_frame_read().frame
#         turn_1 = drone_1_CV.center_subject(img1, 1)

#         #turn
#         if turn_1 == 0: #if no human detected, continue turning
#             turn_1 = 25
#         elif turn_1 == 1: #human centered
#             facing_human = True
#             turn_1 = 0
#         #tello.send_rc_control(0, 0, 0, turn_1)
#         time.sleep(0.3)

#     print("\n\n\nnow moving paths\n\n\n")
#     ##############################
#     #########Path Planning########
#     ##############################
#     drone_1_pos[2] = -1 * tello.get_yaw()

#     while total_time < 60:
#         #update total time
#         total_time = time.time() - start_time

#         #CV
#         img1 = tello.get_frame_read().frame
#         turn_1 = drone_1_CV.center_subject(img1, 1)
#         if turn_1 == 1:
#             turn_1 = 0
        
#         #update timer
#         if iter == 0:
#             sleep_time = 0 #do not update positions for the first loop
#             iter += 1
#         else:
#             sleep_time = time.time() - timer
        
#         if not drone_1_terminate:
#             #calculate new angle
#             print(f"updating position: {drone_1_pos}")
#             drone_1_pos[2] = -1 * tello.get_yaw()

#             #calculate new position
#             theta_x_component_change = 0
#             if drone_1_movement[0] > 0:
#                 theta_x_component_change = -1
#             else:
#                 theta_x_component_change = 1
#             theta_x_component = (drone_1_pos[2] + 90 * theta_x_component_change) % 360
#             theta_y_component = (drone_1_pos[2]) % 360
#             delta_x = abs(drone_1_movement[0]) * math.cos(math.radians(theta_x_component)) + drone_1_movement[1] * math.cos(math.radians(theta_y_component))
#             delta_y = abs(drone_1_movement[0]) * math.sin(math.radians(theta_x_component)) + drone_1_movement[1] * math.sin(math.radians(theta_y_component))
#             drone_1_pos[0] += delta_x * sleep_time
#             drone_1_pos[1] += delta_y * sleep_time

#             #detect collision and manage heights
#             drone_height += go_up * sleep_time
#             print(f"drone height: {drone_height}, normal height: {normal_height}")
#             collision_check = drone_collision.detect_collision(drone_1_pos[0], drone_1_pos[1])
#             if collision_check == "collision":
#                 go_up = 40
#             elif collision_check == "no collision" and drone_height > normal_height * 1.1: #buffer
#                 go_up = -20
#             elif collision_check == "no collision" and drone_height < normal_height * 1.1: #buffer
#                 go_up = 20
#             else:
#                 go_up = 0
            
#             if drone_height > 1.5 * normal_height:
#                 tello.land()
#                 break
            
#             print(f"updated  position: {drone_1_pos}")
#         else:
#             go_up = 0
#             drone_1_pos[2] = -1 * tello.get_yaw()

#         #path planning
#         drone_1_movement = drone_1_path_plan.move_towards_goal(drone_1_pos[0], drone_1_pos[1], drone_1_pos[2], drone_1_terminate)
#         if drone_1_movement[0] == 0.1:
#             drone_1_terminate = True
#             print("\n\n\nnow stopping drone movement\n\n\n")
#             drone_1_movement[0], drone_1_movement[1] = 0, 0
        
#         #move drone and update values, already considered if drone terminated
#         tello.send_rc_control(drone_1_movement[0], drone_1_movement[1], go_up, turn_1)

#         timer = time.time() #time for keeping track of how much to update drones positions

# except KeyboardInterrupt:
#     print("\nManual stop requested. Landing drone...")

# finally:
#     #clean up
#     time.sleep(1)
#     cv2.destroyAllWindows()
#     tello.send_rc_control(0, 0, 0, 0)
#     tello.land()
#     tello.streamoff()
#     tello.end()

