import cv2
import numpy as np
import imutils
from imutils.object_detection import non_max_suppression

class PedestrianHandler:

    def __init__(self):

        self.labelsPath = "coco.names"
        self.LABELS = open(labelsPath).read().strip().split("\n")
        self.weights_path = "yolov4-tiny.weights"
        self.config_path = "yolov4-tiny.cfg"

        self.model = cv2.dnn.readNetFromDarknet(config_path, weights_path)
        self.layer_name = model.getLayerNames()

        self.pedDetected = False

    def pedestrian_detection_procedure(image, model, layer_name, personidz=0):
        (H, W) = image.shape[:2]
        results = []

        blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416),
            swapRB=True, crop=False)
        model.setInput(blob)
        layerOutputs = model.forward(layer_name)

        boxes = []
        centroids = []
        confidences = []

        for output in layerOutputs:
            for detection in output:

                scores = detection[5:]
                classID = np.argmax(scores)
                confidence = scores[classID]

                if classID == personidz and confidence > MIN_CONFIDENCE:

                    box = detection[0:4] * np.array([W, H, W, H])
                    (centerX, centerY, width, height) = box.astype("int")

                    x = int(centerX - (width / 2))
                    y = int(centerY - (height / 2))

                    boxes.append([x, y, int(width), int(height)])
                    centroids.append((centerX, centerY))
                    confidences.append(float(confidence))
        # apply non-maxima suppression to suppress weak, overlapping
        # bounding boxes
        idzs = cv2.dnn.NMSBoxes(boxes, confidences, MIN_CONFIDENCE, NMS_THRESHOLD)
        # ensure at least one detection exists
        if len(idzs) > 0:
            # loop over the indexes we are keeping
            for i in idzs.flatten():
                # extract the bounding box coordinates
                (x, y) = (boxes[i][0], boxes[i][1])
                (w, h) = (boxes[i][2], boxes[i][3])
                # update our results list to consist of the person
                # prediction probability, bounding box coordinates,
                # and the centroid
                res = (confidences[i], (x, y, x + w, y + h), centroids[i])
                results.append(res)
        # return the list of results
        return results

    def detectPedestrian(self, image):
        
        self.layer_name = [layer_name[i[0] - 1] for i in model.getUnconnectedOutLayers()]

        image = imutils.resize(image, width=700)
 
        results = pedestrian_detection(image, model, layer_name, personidz=LABELS.index("person"))

        for res in results:
            cv2.rectangle(image, (res[1][0],res[1][1]), (res[1][2],res[1][3]), (0, 255, 0), 2)
            self.pedDetected = True

        cv2.imshow("Detection",image)

        return self.pedDetected 