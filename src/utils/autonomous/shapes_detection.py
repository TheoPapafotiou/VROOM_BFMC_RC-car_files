import cv2
import numpy as np
import os
#import pytesseract

class ShapesDetection:

    # ==================================== INIT ===================================
    def __init__(self):
        #self.pytesseract.pytesseract.pytesseract_cmd = 'usr\\share\\tesseract-ocr\\tesseract.exe'
        self.conf = "--psm 10"

        self.blue_l = np.array([220, 0, 0])
        self.blue_h = np.array([255, 130, 100])
        self.yellow_l = np.array([0, 180, 220])
        self.yellow_h = np.array([100, 255, 255])
        self.red_l = np.array([0, 0, 220])
        self.red_h = np.array([90, 90, 255])
        self.green_l = np.array([0, 100, 0])
        self.green_h = np.array([100, 255, 90])

    # ========================= FUNCTIONS ===========================================
    def getContours(self, imgContour, imgGray):
        print("Ready to contour")
        imgContour, contours, hierarchy = cv2.findContours(imgContour, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) #CHAIN_APPROX_SIMPLE
        print("I'm fine, thanks")
        circles = cv2.HoughCircles(imgGray, cv2.HOUGH_GRADIENT, 1.1, 100)

        if circles is not None:
            print("Circle detected")
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                cv2.circle(imgContour, (x,y), r, (0, 0, 0), 4)
                cv2.rectangle(imgContour, (x-r, y-r), (x+r, y+r), (0,0,0), 1)
                const_blue = 0
                const_red = 0
                const_green = 0
                const_yellow = 0
                color = "Something"
                for t in range(x-r+30, x+r-30, 10):
                    for k in range(y-r+30, y+r-30, 10):
                        b, g, r = imgContour[k, t]
                        bgr = [b,g,r]
                        if b >= self.blue_l[0] and b <= self.blue_h[0] and g >= self.blue_l[1] and g <= self.blue_h[1] and r >= self.blue_l[2] and r <= self.blue_h[2]:
                            print("HERE_BLUE")
                            const_blue += 1
                        elif b >= self.green_l[0] and b <= self.green_h[0] and g >= self.green_l[1] and g <= self.green_h[1] and r >= self.green_l[2] and r <= self.green_h[2]:
                            print("HERE_GREEN")
                            const_green += 1
                        elif b >= self.red_l[0] and b <= self.red_h[0] and g >= self.red_l[1] and g <= self.red_h[1] and r >= self.red_l[2] and r <= self.red_h[2]:
                            print("HERE_RED")
                            const_red += 1
                        elif b >= self.yellow_l[0] and b <= self.yellow_h[0] and g >= self.yellow_l[1] and g <= self.yellow_h[1] and r >= self.yellow_l[2] and r <= self.yellow_h[2]:
                            print("HERE_YELLOW")
                            const_yellow += 1

                        if const_red == 10:
                            color = "Red"
                            break
                        if const_blue == 10:
                            color = "Blue"
                            break
                        if const_green == 10:
                            color = "Green"
                            break
                        if const_yellow == 10:
                            color = "Yellow"
                            break
                    if color is not "Something":
                        break
                cv2.putText(imgContour, "Shape: Circle", (x - 50, y + 20), cv2.FONT_HERSHEY_COMPLEX, .7, (0, 255, 0), 2)
                cv2.putText(imgContour, "Color: " + color, (x - 50, y + 40), cv2.FONT_HERSHEY_COMPLEX, .7, (0, 255, 0), 2)

        else:
            print("Shape detected")
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 5000:         # to be clarified
                    cv2.drawContours(imgContour, cnt, -1, (0, 0, 0), 7)
                    peri = cv2.arcLength(cnt, True)
                    approx = cv2.approxPolyDP(cnt, 0.02*peri, True)
                    print(len(approx)) # how many angles the shape has
                    x, y, w, h = cv2.boundingRect(approx)
                    if len(approx) == 3:
                        shape = "triangle"
                    elif len(approx) == 4:
                        ar = w / float(h)
                        if ar >= 0.95 and ar <= 1.05:
                            shape = "square"
                        else:
                            shape = "rectangle"
                    elif len(approx) == 5:
                        shape = "pentagon"
                    elif len(approx) == 6:
                        shape = "hexagon"
                    elif len(approx) == 8:
                        shape = "octagon"
                    else:
                        shape = "I don't fucking care"

                    if shape == "square" or shape == "rectangle":
                        gray = imgGray[y+10:y+h-10, x+10:x+w-10]
                        cv2.imshow('cropped' , gray)
                        adaptive_threshold = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 31)  # Last 2 parameters TBQ (STOP:(11,11) & P:(21,31))
                        #print(pytesseract.image_to_string(adaptive_threshold, config = conf))
                        print('############################')

                    else:
                        adaptive_threshold = cv2.adaptiveThreshold(imgGray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 21, 31)  # Last 2 parameters TBQ (STOP:(11,11) & P:(21,31))
                        #print(pytesseract.image_to_string(adaptive_threshold, config = conf))
                        print('!!!!!!!!!!!!!!!!!!!!!!!!')

                    const_blue = 0
                    const_red = 0
                    const_green = 0
                    const_yellow = 0
                    color = "Something"

                    for t in range(x+30,x+w-30, 10):
                        for k in range(y+30, y+h-30, 10):
                            b, g, r = imgContour[k, t]
                            #print(b, g, r)
                            if b >= self.blue_l[0] and b <= self.blue_h[0] and g >= self.blue_l[1] and g <= self.blue_h[1] and r >= self.blue_l[2] and r <= self.blue_h[2]:
                                print("HERE_BLUE")
                                const_blue += 1
                            elif b >= self.green_l[0] and b <= self.green_h[0] and g >= self.green_l[1] and g <= self.green_h[1] and r >= self.green_l[2] and r <= self.green_h[2]:
                                print("HERE_GREEN")
                                const_green += 1
                            elif b >= self.red_l[0] and b <= self.red_h[0] and g >= self.red_l[1] and g <= self.red_h[1] and r >= self.red_l[2] and r <= self.red_h[2]:
                                print("HERE_RED")
                                const_red += 1
                            elif b >= self.yellow_l[0] and b <= self.yellow_h[0] and g >= self.yellow_l[1] and g <= self.yellow_h[1] and r >= self.yellow_l[2] and r <= self.yellow_h[2]:
                                print("HERE_YELLOW")
                                const_yellow += 1

                            if const_red == 10:
                                color = "Red"
                                break
                            if const_blue == 10:
                                color = "Blue"
                                print(color)
                                break
                            if const_green == 10:
                                color = "Green"
                                print(color)
                                break
                            if const_yellow == 10:
                                color = "Yellow"
                                print(color)
                                break
                        if color is not "Something":
                            break

                    cv2.rectangle(imgContour, (x, y), (x + w, y + h), (0, 0, 0), 5)
                    cv2.putText(imgContour, "Shape: " + shape, (x + w - 300, y + 200), cv2.FONT_HERSHEY_COMPLEX, .7, (0, 255, 0), 2)
                    cv2.putText(imgContour, "Color: " + color, (x + w -300, y + 220), cv2.FONT_HERSHEY_COMPLEX, .7, (0, 255, 0), 2)
