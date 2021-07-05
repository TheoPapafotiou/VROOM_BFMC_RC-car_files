import cv2
import numpy as np

class PedestrianDetection:

    def __init__(self):
        print("All signs init")
        self.detections = {
            0:  {
            "class": ['person'],
            "net": cv2.dnn.readNetFromDarknet("src/utils/autonomous/yolov4_tiny.cfg",r"src/utils/autonomous/yolov4-tiny.weights")
            }  
        }

    def detectPedestrianProcedure(self, net, classes, blob, img, height, width):
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
                print("I found a " + label + " fuckin pink girl with confidence " + confidence)
                color = colors[i]
                cv2.rectangle(img,(x,y),(x+w,y+h),color,2)
                cv2.putText(img,label + " " + confidence, (x,y+100),font,2,color,2)
                
                return label, confidence
        
        return "Something", 0.0

    def detectPedestrian(self, img, height, width):
        blob = cv2.dnn.blobFromImage(img, 1/255,(416,416),(0,0,0),swapRB = True,crop= False)
        cfs = 0
        print("I'm working on the Pedestrian class")
        label = "Something"
        confidence = 0.0
        label, confidence = self.detectPedestrianProcedure(
            self.detections[cfs]['net'],
            self.detections[cfs]['class'],
            blob,
            img,
            height,
            width
        )
        print(label, confidence)
        return label, confidence
