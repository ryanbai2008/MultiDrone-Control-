import cv2
import numpy as np

class CV:
    def __init__(self, num, cfg_path="yolo_model/yolov3-tiny.cfg", weights_path="yolo_model/yolov3-tiny.weights"):
        self.cfg_path = cfg_path
        self.weights_path = weights_path
        self.net = cv2.dnn.readNetFromDarknet(cfg_path, weights_path)
        with open("yolo_model/coco.names", "r") as f:
            self.classes = [line.strip() for line in f.readlines()]
        self.layer_names = self.net.getLayerNames()
        self.output_layers = [self.layer_names[i - 1] for i in self.net.getUnconnectedOutLayers()]


        self.size = (1280, 720)
        self.out = cv2.VideoWriter('telloVid' + str(num) + '.avi',  
                         cv2.VideoWriter_fourcc(*'MJPG'), 
                         30, self.size) 
    
    def center_subject(self, img, drone_number):
        height, width, _ = img.shape
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        small = cv2.resize(img, (416, 416), interpolation=cv2.INTER_LINEAR)

        blob = cv2.dnn.blobFromImage(small, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)


 
        centers = []

        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores) # Index of predicted class
                confidence = scores[class_id]

                if confidence > 0.4 and class_id == 0:  # Filter for class "person"
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)

                    centers.append((center_x, center_y))


        # Resize image for faster display
        # dim = (int(width / 4), int(height / 4))
        # new_img = cv2.resize(img, dim)
        # window_name = f"img{drone_number}"
        # cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)  # Allow manual resizing
        # cv2.resizeWindow(window_name, dim[0], dim[1])  # Set window to match the resized image

        if len(centers) > 0:
            avg_center_x = 0
            for center in centers:
                avg_center_x += center[0]
            avg_center_x /= len(centers)

            # Return movement based on object position
            if avg_center_x > width / 2 + 40:
                #15, 35 before
                return int(max(10, min(30, 1.02 ** (abs(avg_center_x - width / 2))))) #turn right, scales based on how far from center subject is
            elif avg_center_x < width / 2 - 40:
                return -1 * int(max(10, min(30, 1.02 ** (abs(avg_center_x - width / 2))))) #turn left  # Turn left
            else:
                return 1  # Go straight

        self.out.write(img)
        print("\n\n\n\nimage written\n\n\n\n")
      
        return 0
      
    def record_video_stream(self, img):
        try:
            #frame = img.frame
            self.out.write(img)
        except:
            print('nuh uh')
        
