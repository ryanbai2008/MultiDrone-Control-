import cv2
import numpy as np

class CV:
    def __init__(self, cfg_path="yolo_model/yolov3-tiny.cfg", weights_path="yolo_model/yolov3-tiny.weights"):
        self.cfg_path = cfg_path
        self.weights_path = weights_path
        self.net = cv2.dnn.readNet(weights_path, cfg_path, framework='Darknet')
        with open("yolo_model/coco.names", "r") as f:
            self.classes = [line.strip() for line in f.readlines()]
        self.layer_names = self.net.getLayerNames()
        self.output_layers = [self.layer_names[i - 1] for i in self.net.getUnconnectedOutLayers()]
    
    def center_subject(self, img, drone_number):
        height, width, _ = img.shape
        #img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)

        class_ids = []
        confidences = []
        boxes = [] 
        centers = []

        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores) # Index of predicted class
                confidence = scores[class_id]

                if confidence > 0.5 and class_id == 0:  # Filter for class "person"
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)

                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    boxes.append([x, y, w, h])
                    centers.append((center_x, center_y))
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        # Resize image for faster display
        dim = (int(width / 4), int(height / 4))
        new_img = cv2.resize(img, dim)
        window_name = f"img{drone_number}"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)  # Allow manual resizing
        cv2.resizeWindow(window_name, dim[0], dim[1])  # Set window to match the resized image

        if len(confidences) > 0:
            max_index = np.argmax(confidences)
            x, y, w, h = boxes[max_index]
            center_x, center_y = centers[max_index]

            # Scale bounding box coordinates based on resized image
            scale_x = new_img.shape[1] / float(width)
            scale_y = new_img.shape[0] / float(height)
            x = int(x * scale_x)
            y = int(y * scale_y)
            w = int(w * scale_x)
            h = int(h * scale_y)

            # Draw rectangle and add text
            cv2.rectangle(new_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            label = str(self.classes[class_ids[max_index]])
            confidence = str(round(confidences[max_index], 2))
            cv2.putText(new_img, f"{label} {confidence}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            
            # Display the image
            cv2.imshow(window_name, new_img)
            cv2.waitKey(1)
            # Return movement based on object position
            if center_x > width / 2 + 80:
                #15, 35 before
                return int(max(10, min(30, 1.02 ** (abs(center_x - width / 2))))) #turn right, scales based on how far from center subject is
            elif center_x < width / 2 - 80:
                return -1 * int(max(10, min(30, 1.02 ** (abs(center_x - width / 2))))) #turn left  # Turn left
            else:
                return 1  # Go straight

        cv2.imshow("img" + str(drone_number), img)
        cv2.waitKey(1)
        return 0

