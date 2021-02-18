import cv2
import numpy as np
import imutils

class Tracker:
    
    def track_object(img, timestamp):
        tracker = cv2.TrackerMOSSE_create()    # high speed, low accuracy
        #tracker = cv2.TrackerCSRT_create()      # low speed, high accuracy

        timer = cv2.getTickCount()

        bbox = cv2.selectROI(img)

        tracker.init(img, bbox)

        success, bbox = tracker.update(img)
        print(bbox)

        if success:
            drawBox(img)

        else:
            cv2.putText(img, "Lost", (75,75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 3)

        fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
        cv2.putText(img, str(int(fps)), (75,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 3)
        cv2.putText(img, "Tracking", (75, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 3)
        cv2.imshow("Tracking" , img)
        
        cv2.imwrite(str(timestamp)+".jpg", img)

