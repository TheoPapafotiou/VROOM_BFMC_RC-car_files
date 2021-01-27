import cv2
import numpy as np

class SignDetection:

    def __init__(self):
        print("All signs init")
        self.detections = {
            0: {
                "class": ['Parking_Spot', 'Pedestrians', 'Ahead'],
                "net": cv2.dnn.readNetFromDarknet("src/utils/autonomous/yolov3_tiny-custom.cfg",r"src/utils/autonomous/weights_tiny/yolov3_tiny-custom_blueSigns.weights")
            },
            1: {
                "class": ['Stop', 'No_entry', 'Roundabout_mandatory'],
                "net": cv2.dnn.readNetFromDarknet("src/utils/autonomous/yolov3_tiny-custom.cfg",r"src/utils/autonomous/weights_tiny/yolov3_tiny-custom_redSigns.weights")
            },
            2: {
                "class": ['Highway_End', 'Highway_Start'],
                "net": cv2.dnn.readNetFromDarknet("src/utils/autonomous/yolov2_tiny-custom.cfg",r"src/utils/autonomous/weights_tiny/yolov2_tiny-custom_greenSigns.weights")
            },
            3: {
                "class": ['Priority_road'],
                "net": cv2.dnn.readNetFromDarknet("src/utils/autonomous/yolov1_tiny-custom.cfg",r"src/utils/autonomous/weights_tiny/yolov1_tiny-custom_prioritySigns.weights")
            },
            4: {
                "class": ['Traffic_Light'],
                "net": cv2.dnn.readNetFromDarknet("src/utils/autonomous/yolov1_tiny-custom.cfg",r"src/utils/autonomous/weights_tiny/yolov1_tiny-custom_trafficLights.weights")
            }
        }

    def detectSignProcedure(self, net, classes, blob, img, hight, width):
        print(classes)
        net.setInput(blob)
        output_layers_name = net.getUnconnectedOutLayersNames()
        layerOutputs = net.forward(output_layers_name)

        boxes =[]
        confidences = []
        class_ids = []

        for output in layerOutputs:
            for detection in output:
                score = detection[5:]
                class_id = np.argmax(score)
                confidence = score[class_id]
                if confidence > 0.5:
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * hight)
                    w = int(detection[2] * width)
                    h = int(detection[3]* hight)
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
                print("I found a " + label + " sign with confidence " + confidence)
                color = colors[i]
                cv2.rectangle(img,(x,y),(x+w,y+h),color,2)
                cv2.putText(img,label + " " + confidence, (x,y+100),font,2,color,2)


    def detectSign(self, img, hight, width, countFours):
        blob = cv2.dnn.blobFromImage(img, 1/255,(416,416),(0,0,0),swapRB = True,crop= False)
        cfs = 1#int((countFours - 1) / 2)
        print(cfs)
        self.detectSignProcedure(
            self.detections[cfs]['net'],
            self.detections[cfs]['class'],
            blob,
            img,
            hight,
            width
        )
