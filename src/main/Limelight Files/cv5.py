import cv2
import numpy as np

# INSTRUCTIONS:
# Change pipeline type to Python and put this as the python code
# Detects cones and cubes, as well as estimates distances to within ~5cm usually
# Check Limelight-OpenCV branch for code that works with this

# Also the limelight subsystem is broken and had to be modified

# global variables go here:
min_gamepeice_area = 8000

lower_cone = (10, 130, 100)
upper_cone = (55, 255, 255)
lower_cube_dark = (123, 165, 40)
uppder_cube_dark = (145, 255, 255)
lower_cube_light = (120, 1, 160)
uppder_cube_light = (145, 255, 255)


def pixelInRange(centerPixelColor, lower_bound, upper_bound):
    if centerPixelColor[0] >= lower_bound[0] and centerPixelColor[0] <= upper_bound[0]: # Check hue
        if centerPixelColor[1] >= lower_bound[1] and centerPixelColor[1] <= upper_bound[1]: # Check saturation
            if centerPixelColor[2] >= lower_bound[2] and centerPixelColor[2] <= upper_bound[2]: # Check value
                return True
    return False
    

# runPipeline() is called every frame by Limelight's backend.
def runPipeline(image, llrobot):
    llpython = [] # Array to return to shuffleboard
    largestContour = np.array([[]]) # <-- Ignore this

    #Combine colour thresholds
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    img_cone_threshold = cv2.inRange(img_hsv, lower_cone, upper_cone)
    img_cube_threshold = cv2.inRange(img_hsv, lower_cube_dark, uppder_cube_dark)
    img_cube_highlight_threshold = cv2.inRange(img_hsv, lower_cube_light, uppder_cube_light)

    img_contour_threshold = cv2.bitwise_or(img_cone_threshold, img_cube_threshold)
    img_contour_threshold = cv2.bitwise_or(img_contour_threshold, img_cube_highlight_threshold)

    #Find Contours
    contours, hierarchy  = cv2.findContours(img_contour_threshold,
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        cv2.drawContours(image, contours, -1, 255, 2)
        for i, cont in enumerate(contours):
            area = cv2.contourArea(cont)
            if area > min_gamepeice_area:
                
                rect = cv2.minAreaRect(cont)
                box = cv2.boxPoints(rect)
                box = np.intp(box)

                cv2.drawContours(image, [box], -1,(0,255,0),5)

                x,y,w,h = cv2.boundingRect(cont)
                cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,255),5)

                centerPixelColor = img_hsv[int(y + h/2)][int(x + w/2)]
                if pixelInRange(centerPixelColor, lower_cone, upper_cone):
                    angle = rect[-1]

                    # Limelight networtables stuff
                    llpython.append(0) # Cone / Cube ID

                    if w < h:
                        llpython.append(1) # Upright Cone
                        cv2.putText(image, #Debug
                            "Upright",
                            (x, y+60),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1.0, (0, 0, 255),2, cv2.LINE_AA)
                    else:
                        llpython.append(0) # Fallen Cone
                        cv2.putText(image, #Debug
                            "Fallen",
                            (x, y+60),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1.0, (0, 0, 255),2, cv2.LINE_AA)
                    

                    #Calculate distance
                    distance = 16452*pow(area, -0.516)

                    #Debug
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
                else:

                    # Limelight networtables stuff

                    llpython.append(1) # Cone / Cube ID
                    llpython.append(0) # Dummy Data value for parsing purposes

                    #Calculate distance
                    distance = 24143*pow(area, -0.546)

                    #Debug
                    cv2.putText(image,
                        "Cube Dist: ",
                        (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (0, 0, 255),2, cv2.LINE_AA)
                    cv2.putText(image,
                        str(round(distance)) + " cm",
                        (x, y+30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (0, 0, 255),2, cv2.LINE_AA)
                    
                llpython.append(distance)


    return largestContour, image, llpython

