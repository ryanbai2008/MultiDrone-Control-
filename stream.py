from customtello import myTello
import cv2
import time
import tello_tracking

# Initialize drone
tello = myTello("192.168.10.2", 11111)  # Change the IP based on your setup
drone_1_CV = tello_tracking.CV()

tello2 = myTello("192.168.10.3", 11111)  # Change the IP based on your setup
drone_2_CV = tello_tracking.CV()


# Connect to drone
tello2.connect()

# Allow some time for frames to start coming in

# Connect to drone
tello.connect()

# Start video streaming
tello.streamon()
tello.start_video_stream(1)

# Start video streaming
tello2.streamon()
tello2.start_video_stream(2)
# Allow some time for frames to start coming in
time.sleep(2)

# Main loop to display the video stream
while True:
    frame = tello.get_frame_read()
    frame2 = tello2.get_frame_read()

    if frame is not None:
        drone_1_CV.center_subject(frame, 1)
    else:
        print("No frame recieved from drone 1")

        
    if frame2 is not None:
        drone_2_CV.center_subject(frame2, 2)
    else:
        print("No frame recieved from drone 2")

    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
tello.end()
cv2.destroyAllWindows()
