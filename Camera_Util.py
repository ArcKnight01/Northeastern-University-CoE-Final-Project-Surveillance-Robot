import cv2
import matplotlib.pyplot as plt
from matplotlib import cm
import sys
from numpy.core.numeric import ones
from time import sleep

import os
try:
    nodeName= str(os.uname().nodename)
except:
    import platform
    nodeName = str(platform.uname().node)
print(nodeName)
if (nodeName == "terminatorpi") or (nodeName == 'robotpi'):
    import apriltag
    plotting = False
    supported = True
else:
    plotting = True
    supported = False

import numpy as np

def sensor_position(pix_x, pix_y, res_x, res_y):
    sensor_width,sensor_height = (0.00368, 0.00276) #mm to meters
    origin = (res_x/2,res_y/2)
    ratio_x, ratio_y = (sensor_width/res_x, sensor_height/res_y)
    
    pix_x, pix_y = (pix_x - origin[0], pix_y - origin[1])

    sensor_pos_x, sensor_pos_y = (ratio_x*pix_x, ratio_y*pix_y)
    
    return (sensor_pos_x, sensor_pos_y)


focal_length = 0.00304 #mm is the focal length

def sensor_angle(sensor_pos_x, sensor_pos_y, f):
    return np.degrees(np.arctan2(sensor_pos_x,f))

def detect_apriltags(image):
    # image = cv2.imread('/home/pi/NuRobotics/Frames/tag16_05_00000.png')
    
    #convert image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    options = apriltag.DetectorOptions(families=["tag36h11","tag16h5"])
    detector = apriltag.Detector(options)
    
    results = detector.detect(gray)
    centers = []
    angles = []
    corners = []
    tagFamilies = []
    tagIds = []
    tag_detected = False
    for r in results:
        #find the corners of the april tag
        (ptA, ptB, ptC, ptD) = r.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))
        
        corners.append((ptA, ptB, ptC, ptD))
        #find the centers
        center = (int(r.center[0]), int(r.center[1]))
        centers.append(center)
        
        #find the angles 
        a = sensor_position(center[0], center[1], image.shape[1], image.shape[0])
        angle = sensor_angle(a[0], a[1], focal_length)
        angles.append(angle)

        print(f"[INFO] tag center @ {center}")
        print(f"[INFO] tag angle @ {angle}")
        # draw the bounding box of the AprilTag detection
        cv2.line(image, ptA, ptB, (0, 255, 0), 2)
        cv2.line(image, ptB, ptC, (0, 255, 0), 2)
        cv2.line(image, ptC, ptD, (0, 255, 0), 2)
        cv2.line(image, ptD, ptA, (0, 255, 0), 2)

       
        # draw the center (x, y)-coordinates of the AprilTag
        cv2.circle(image, (center[0], center[1]), 5, (0, 0, 255), -1)
        
        # draw the tag family on the image
        try:
            tagFamily = r.tag_family.decode("utf-8")
        except:
            tagFamily = r.tag_family
        try:
            tagId = r.tag_id.decode("utf-8")
        except:
            tagId = r.tag_id
        cv2.putText(image, tagFamily, (ptA[0], ptA[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        print(f"[INFO] tag family: {tagFamily}")
        tagFamilies.append(tagFamily)
        print(f"[INFO tag id {tagId}]")
        tagIds.append(tagId)
    
    tag_detected = True if len(results) != 0 else False
    return tag_detected, image, tagFamilies, tagIds, centers, angles, corners
    

def find_centers(filter_image, rgb_image, thresh):
    object_detection_surface = cv2.boxFilter(filter_image.astype(int), -1, (30, 30), normalize=False)
    exitCode = False
    if np.max(object_detection_surface) <= 0:
        exitCode = True

    object_detection_surface = object_detection_surface * 255/np.max(object_detection_surface)
    # threshold = thresh * 255 / np.max(object_detection_surface)
    
    img8 = object_detection_surface.astype(np.uint8)

    threshold, img_out = cv2.threshold(img8, thresh, 255, cv2.THRESH_BINARY)

    if cv2.__version__ == '3.2.0':
        _, contours, hierarchy = cv2.findContours(img_out, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    else:
        contours, hierarchy = cv2.findContours(img_out, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    if(exitCode == True):
        return [], [], object_detection_surface, img_out
    centers = []
    angles = []
    for contour in contours:
        max = np.max([item[0][0] for item in contour])

        center = np.mean(contour, axis = 0)[0,:]
        centers.append(center)
        
        a = sensor_position(center[0], center[1], rgb_image.shape[1], rgb_image.shape[0])
        b = sensor_angle(a[0], a[1], focal_length)
        
        angles.append(b)
    return centers, angles, object_detection_surface, img_out

def get_ranges(red_range, green_range, blue_range, rgb_image):
    rgb_filt = cv2.boxFilter(rgb_image, -1, (9,9))
    
    red_filt = rgb_filt[:,:,0]
    green_filt = rgb_filt[:,:,1]
    blue_filt = rgb_filt[:,:,2]

    img_thresh_red = np.logical_and(red_filt > red_range[0], red_filt < red_range[1])
    img_thresh_green = np.logical_and(green_filt > green_range[0], green_filt < green_range[1])
    img_thresh_blue = np.logical_and(blue_filt > blue_range[0], blue_filt < blue_range[1])

    img_thresh_RG = np.logical_and(img_thresh_red, img_thresh_green)
    img_thresh_RGB = np.logical_and(img_thresh_RG, img_thresh_blue)

    return(img_thresh_RGB)

def detect_buoys(img):
    # hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # rgb_image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    rgb_image = np.flip(img, axis=2) 
    # rgb_image = np.flip(rgb_image, 0)
    r_red_range = (110,255)
    r_green_range = (0,50)
    r_blue_range = (0,50)
    img_thresh_red = get_ranges(r_red_range, r_green_range, r_blue_range, rgb_image)
    reds_centers, reds_angles, object_detection_surface, img_out = find_centers(img_thresh_red, rgb_image, 50)
    print(reds_angles, reds_centers, object_detection_surface, img_out)
    return reds_angles, reds_centers, object_detection_surface, img_out

# #comment out the below when not testing camera:
if (__name__=='__main__') & (True):
    fig, ax = plt.subplots(1,3)
    repeat=True
    while(repeat==True):
        for frame_num in range(1681905135, 1681908011):
            
            img = cv2.imread(f'./Frames/frame_{frame_num}.jpg') 
            if img is not None:
                r_angles, r_centers, object_detection_surface, img_out = detect_buoys(img)
                if(len(r_angles)!=0):
                    print("Detected")
                else:
                    print("Not detected")
                img = np.flip(img, axis=2)
                print(frame_num)
                print('\n')
                print(r_angles)
                ax[0].clear()
                ax[0].imshow(img)
                ax[1].clear()
                ax[1].imshow(object_detection_surface)
                ax[2].clear()
                ax[2].imshow(img_out)
                print(r_centers)
                if len(r_centers) != 0:
                    ax[0].plot(r_centers[0][0], r_centers[0][1], 'ro')
                    ax[1].plot(r_centers[0][0], r_centers[0][1], 'ro')
                    ax[2].plot(r_centers[0][0], r_centers[0][1], 'ro')
                plt.pause(2)
                
                plt.draw()
                
