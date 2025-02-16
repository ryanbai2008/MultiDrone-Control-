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
    def center_subject(self, img):
        height, width, _ = img.shape
        blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)
        class_ids = []
        confidences = []
        boxes = [] 
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                    
                if confidence > 0.5 and class_id == 0:
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)

                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                        
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)
            
        indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
            
        if len(indices) > 0:
            for i in indices.flatten():
                x, y, w, h = boxes[i]
                label = str(self.classes[class_ids[i]])
                confidence = str(round(confidences[i], 2))

                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(img, label + " " + confidence, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            
        cv2.imshow("Human Detection and Tracking with Tello", img)
        avg_center = [0, 0] #(X,Y)
        count = 0
        for i in indices.flatten():
            avg_center[0] += boxes[0] + w // 2
            avg_center[1] += boxes[1] + h // 2
            count += 1
        avg_center[0] /= count
        avg_center[1] /= count
        turn_left = 0
        if avg_center[0] > (width + 50) / 2: #has to be certain amount past center to call for adjustments
            turn_left = -1 * min(5, 1.05 ** (avg_center[0] - width / 2)) #############################adjust for magnitude of turning, perhaps include distance, adjusting for move_left vals in path_planner 
#######################################################################################################will also have an effect
        elif avg_center[0] < (width - 50) / 2: #same here     #######################this returns the angle it should rotate, adjust this in main.py to become velocity for smoothness
            turn_left = 1 * min(5, 1.05 ** (avg_center[0] - width / 2))
        return turn_left

    cv2.destroyAllWindows()
