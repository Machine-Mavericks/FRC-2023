import cv2
import numpy as np
import math

# global variables go here:
min_gamepeice_area = 8000
# cone_height_cm = 33.2
# ll_mountpoint_x = 1
# ll_mountpoint_y = 1

lower_cone = (10, 130, 100)
upper_cone = (55, 255, 255)

def pixelInRange(centerPixelColor, lower_bound, upper_bound):
    if centerPixelColor[0] >= lower_bound[0] and centerPixelColor[0] <= upper_bound[0]: # Check hue
        if centerPixelColor[1] >= lower_bound[1] and centerPixelColor[1] <= upper_bound[1]: # Check saturation
            if centerPixelColor[2] >= lower_bound[2] and centerPixelColor[2] <= upper_bound[2]: # Check value
                return True
    return False


# runPipeline() is called every frame by Limelight's backend.
def runPipeline(image, llrobot):
    #return
    llpython = []
    largestContour = np.array([[]])

    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    img_cone_threshold = cv2.inRange(img_hsv, lower_cone, upper_cone)

    
    contours, hierarchy  = cv2.findContours(img_cone_threshold,
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        cv2.drawContours(image, contours, -1, 255, 2)
        for i, cont in enumerate(contours):
            area = cv2.contourArea(cont)
            if area > min_gamepeice_area:
                largestContour = max(contours, key=cv2.contourArea)
                
                rect = cv2.minAreaRect(cont)
                box = cv2.boxPoints(rect)
                box = np.intp(box)


                
                cv2.drawContours(image, [box], -1,(0,255,0),5)


                x,y,w,h = cv2.boundingRect(cont)
                cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,255),5)

                #Calculate distance
                distance = 17259*pow(area, -0.504)

                #print("Dist " + str(distance) + " H: " + str(h))


                
                #distXfromCentre = (x+w/2) - (image.shape[1] / 2)

                # ratio = cone_height_cm / h
                
                # ll_relative_x = distXfromCentre

                # ll_relative_y = math.sqrt(distance^2 - ll_relative_x^2)

                # robot_relative_x = ll_relative_x + ll_mountpoint_x
                # robot_relative_y = ll_relative_y + ll_mountpoint_y
                
                # llpython.append(distance)
                # llpython.append(robot_relative_x)
                # llpython.append(robot_relative_y)

                llpython.append(distance)







                cv2.putText(image,
                        "Cone Dist: ",
                        (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (255, 0, 0),2, cv2.LINE_AA)
                cv2.putText(image,
                    str(round(distance)) + " cm",
                    (x, y+30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0, (255, 0, 0),2, cv2.LINE_AA)

                if w < h:
                        #llpython.append(1) # Upright Cone
                        cv2.putText(image,
                            "Upright",
                            (x, y+60),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1.0, (0, 0, 255),2, cv2.LINE_AA)
                else:
                     #llpython.append(0) # Fallen Cone
                        cv2.putText(image,
                            "Fallen",
                            (x, y+60),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1.0, (0, 0, 255),2, cv2.LINE_AA)
             

                

    return largestContour, image, llpython