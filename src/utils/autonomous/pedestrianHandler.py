import cv2
import numpy as np
import imutils
from imutils.object_detection import non_max_suppression

class PedestrianHandler:

    def __init__(self):

            self.hog = cv2.HOGDescriptor()
            self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
            self.bbox = ()
            self.pedDetected = False
            self.stepSize = (4,4)
            self.padding = (4, 4)
            self.scale = 1.05
            self.suppresionThresh = 0.65

    def drawBox(self, img):
        x, y, w, h = int(self.bbox[0]), int(self.bbox[1]), int(self.bbox[2]), int(self.bbox[3])
        cv2.rectangle(img, (x,y), ((x+w),(y+h)), (255, 0, 255), 3, 1)

    def detectPedestrian(self, img):
        
        img = imutils.resize(img, width=min(400, img.shape[1]))

        (regions, _) = self.hog.detectMultiScale(img, winStride=self.stepSize, padding=self.padding, scale=self.scale)

        for(x, y, w, h) in regions:
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
            self.pedDetected = True

        regions = np.array([[x, y, x + w, y + h] for (x, y, w, h) in regions])
        pick = non_max_suppression(regions, probs=None, overlapThresh=suppresionThresh)

        for(xA, yA, xB, yB) in pick:
            cv2.rectangle(img, (xA, yA), (xB, yB), (0, 255, 0), 2)

        return self.pedDetected 