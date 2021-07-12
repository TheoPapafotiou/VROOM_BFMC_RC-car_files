import cv2
import numpy as np

class SignDetection:

    def __init__(self):
        print("All signs init")
        self.detections = {
            0: {
            "class": ['ParkingSpot','Pedestrian','Ahead','HighayEnd','HighwayStart','PriorityRoad','Stop','NoEntry','Roundabout','TrafficLights'],
            "net": cv2.dnn.readNetFromDarknet("src/utils/autonomous/yolov3_tiny-custom.cfg",r"src/utils/autonomous/weights_tiny/yolov3_tiny-custom_total.weights")
            }  
        }

        ### Distance params ###
        self.init_distance = 24.0 #actual distance from object (cm)
        self.known_width_sign = 11.0  #actual width of the object (cm)
        self.init_pixels = 250  #perceived width in pixels (pt)

        self.focal_length = (self.init_pixels * self.init_distance) / self.known_width_sign
        

    # ================================== DISTANCE TO CAMERA ==============================
    def distance_to_camera(knownWidth, focalLength, pixelsWidth):
        # compute and return the distance from the maker to the camera
        return (knownWidth * focalLength) / pixelsWidth


    def detectSignProcedure(self, net, classes, blob, img, height, width):
        net.setInput(blob)
        output_layers_name = net.getUnconnectedOutLayersNames()
        layerOutputs = net.forward(output_layers_name)

        boxes =[]
        confidences = []
        class_ids = []
        print("In the procedure")

        for output in layerOutputs:
            for detection in output:
                score = detection[5:]
                class_id = np.argmax(score)
                confidence = score[class_id]
                if confidence > 0.3:
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3]* height)
                    x = int(center_x - w/2)
                    y = int(center_y - h/2)
                    boxes.append([x,y,w,h])
                    confidences.append((float(confidence)))
                    class_ids.append(class_id)

        indexes = cv2.dnn.NMSBoxes(boxes,confidences,.8,.4)
        font = cv2.FONT_HERSHEY_PLAIN
        colors = np.random.uniform(0,255,size =(len(boxes),3))
        if  len(indexes)>0:
            for i in indexes.flatten():
                x,y,w,h = boxes[i]
                label = str(classes[class_ids[i]])
                confidence = str(round(confidences[i],2))
                distance = self.distance_to_camera(self.focal_length, w, self.known_width_sign)
                print("I found a " + label + " sign with confidence " + confidence)
                color = colors[i]
                cv2.rectangle(img,(x,y),(x+w,y+h),color,2)
                cv2.putText(img,label + " " + confidence, (x,y+100),font,2,color,2)
                
                return label, confidence, distance
        
        return "Something", 0.0, 0.0

    def detectSign(self, img, height, width):
        blob = cv2.dnn.blobFromImage(img, 1/255,(416,416),(0,0,0),swapRB = True,crop= False)
        cfs = 0
        #print("I'm working on the class")
        label = "Something"
        confidence = 0.0
        distance = 0.0
        label, confidence, distance = self.detectSignProcedure(
            self.detections[cfs]['net'],
            self.detections[cfs]['class'],
            blob,
            img,
            height,
            width
        )
        print(label, confidence, distance)
        return label, confidence, distance
