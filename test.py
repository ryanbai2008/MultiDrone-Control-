import cv2
import socket
import numpy as np

# Define the Tello UDP stream address
UDP_IP = "0.0.0.0"  # Listen on any IP address
UDP_PORT = 11111     # Tello sends video on port 11111

# Set up the socket to receive video data
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

# Tello video stream starts on port 11111, so we need to send a command to begin video stream
# Send command to start video stream (required)
start_video_command = b'command'
sock.sendto(start_video_command, ("192.168.10.1", 8889))  # Tello command port

# Set up a buffer to store the incoming video data
frame_buffer = b''

# Create a window to display the video stream
cv2.namedWindow("Tello Stream", cv2.WINDOW_NORMAL)

while True:
    try:
        # Receive the video stream
        data, addr = sock.recvfrom(2048)  # Receive the video data, buffer size of 2048 bytes
        
        # Append the received data to the frame buffer
        frame_buffer += data

        # Check for the end of a frame (H.264 data will have a specific frame header/footer pattern)
        # For simplicity, here we'll check if we've received a full frame by using a dummy check
        if len(frame_buffer) > 1024:  # Example check for frame length, adjust as needed
            # Decode the frame from the byte buffer into an image
            nparr = np.frombuffer(frame_buffer, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            if frame is not None:
                # Show the frame using OpenCV
                cv2.imshow("Tello Stream", frame)

                # Break the loop if 'q' is pressed
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            # Clear the buffer to begin receiving the next frame
            frame_buffer = b''

    except KeyboardInterrupt:
        break

# Clean up and close the window
cv2.destroyAllWindows()
sock.close()
