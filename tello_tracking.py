import cv2
import numpy as np

class CV:
    def __init__(self, cfg_path = "yolo_model/yolov3.cfg", weights_path = "yolo_model/yolov3.weights"):
        self.cfg_path = "yolo_model/yolov3.cfg"
        self.weights_path = "yolo_model/yolov3.weights"
        self.net = cv2.dnn.readNet(weights_path, cfg_path, framework='Darknet')
        with open("yolo_model/coco.names", "r") as f:
            self.classes = [line.strip() for line in f.readlines()]
            self.layer_names = self.net.getLayerNames()
            self.output_layers = [self.layer_names[i - 1] for i in self.net.getUnconnectedOutLayers()]
    
    
    #img = tello.get_frame_read().frame
    def center_subject(self, img, drone_number):
        height, width, _ = img.shape
        blob = cv2.dnn.blobFromImage(img, 0.00392, (256, 256), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers) #[x_center, y_center, width, height, confidence, class_1_prob, class_2_prob, ..., class_n_prob]

        class_ids = []
        confidences = []
        boxes = [] 
        centers = []

        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores) #index of predict
                confidence = scores[class_id]
                    
                if confidence > 0.5 and class_id == 0:
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)

                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                        
                    boxes.append([x, y, w, h])
                    centers.append(center_x, center_y)
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        if len(confidences) > 0:
            max_index = np.argmax(confidences)
            max_confidence = confidences[max_index]
            x, y, w, h = boxes[max_index]
            center_x, center_y = centers[max_index]
            label = str(self.classes[class_ids[max_index]])

            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(img, label + " " + max_confidence, (x, y - 10), cv2.FONT_HERSHEY_PLAIN, 0.9, (0, 255, 0), 2)
            cv2.imshow("img" + str(drone_number), img)

            if center_x > width / 2 + 75:
                return max(25, min(50, 1.035 ** (abs(center_x - width / 2)))) #turn right, scales based on how far from center subject is
            elif center_x < width / 2 - 75:
                return -1 * max(25, min(50, 1.035 ** (abs(center_x - width / 2)))) #turn left
            
        cv2.imshow("img" + str(drone_number), img)
        return 0
