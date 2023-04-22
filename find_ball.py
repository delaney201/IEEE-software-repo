#!/usr/bin/env python

"""
ON THE RASPI: roslaunch raspicam_node camerav2_320x240.launch enable_raw:=true

   0------------------> x (cols) Image Frame
   |
   |        c    Camera frame
   |         o---> x
   |         |
   |         V y
   |
   V y (rows)


SUBSCRIBES TO:
    /raspicam_node/image: Source image topic
    
PUBLISHES TO:
    /blob/image_blob : image with detected blob and search window
    /blob/image_mask : masking    
    /blob/point_blob : blob position in adimensional values wrt. camera frame

"""


#--- Allow relative importing
if __name__ == '__main__' and __package__ is None:
    from os import sys, path
    sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
    
import sys
import rospy
import cv2
import time
import numpy as np
import RPi.GPIO as GPIO

from std_msgs.msg           import String
from sensor_msgs.msg        import Image
from geometry_msgs.msg      import Point
from std_msgs.msg import Int8, Float32
from cv_bridge              import CvBridge, CvBridgeError
from include.blob_detector  import *

IN3 = 37


def display_img(img, delay=1000):
    """
    One liner that displays the given image on screen
    """
    cv2.namedWindow("Vid", cv2.WINDOW_AUTOSIZE)
    cv2.imshow("Vid", img)
    cv2.waitKey(delay) 
    
def grey_histogram(img, nBins=64):
    """
    Returns a one dimension histogram for the given image
    The image is expected to have one channel, 8 bits depth
    nBins can be defined between 1 and 255 
    """
    hist_size = [nBins]
    h_ranges = [0, 255]
    hist = cv2.CreateHist(hist_size , cv2.HIST_ARRAY, [[0, 255]], 1)
    cv2.CalcHist([img], hist)

    return hist

def extract_bright(grey_img, histogram=False):
    """
    Extracts brightest part of the image.
    Expected to be the LEDs (provided that there is a dark background)
    Returns a Thresholded image
    histgram defines if we use the hist calculation to find the best margin
    """
    ## Searches for image maximum (brightest pixel)
    # We expect the LEDs to be brighter than the rest of the image
    [minVal, maxVal, minLoc, maxLoc] = cv2.minMaxLoc(grey_img)
    print( "Brightest pixel val is %d" %(maxVal))
    
    #We retrieve only the brightest part of the image
    # Here is use a fixed margin (80%), but you can use hist to enhance this one    
    if 0:
        ## Histogram may be used to wisely define the margin
        # We expect a huge spike corresponding to the mean of the background
        # and another smaller spike of bright values (the LEDs)
        hist = grey_histogram(grey_img, nBins=64)
        [hminValue, hmaxValue, hminIdx, hmaxIdx] = cv2.getMinMaxHistValue(hist)
        margin = 0# statistics to be calculated using hist data    
    else:  
        margin = 0.8
        
    thresh = int( maxVal * margin) # in pix value to be extracted
    print( "Threshold is defined as %d" %(thresh))

    width = grey_img.shape[1]
    height = grey_img.shape[0]
    dim = (width, height)

    thresh_img = cv2.resize(grey_img, dim, 1)
    cv2.threshold(grey_img, 127, 255, cv2.THRESH_BINARY)
    
    return thresh_img

def find_leds(thresh_img):
    """
    Given a binary image showing the brightest pixels in an image, 
    returns a result image, displaying found leds in a rectangle
    """
    contours, darn = cv2.findContours(thresh_img, # cv2.CreateMemStorage(), 
                               mode=cv2.RETR_EXTERNAL , 
                               method=cv2.CHAIN_APPROX_NONE , 
                               offset=(0, 0))
    
    return contours


def leds_positions(regions):
    """
    Function using the regions in input to calculate the position of found leds
    """
    centers = []
    for x, y, width, height in regions:
        centers.append( [x+ (width / 2),y + (height / 2)])

    return centers


class BlobDetector:

    def __init__(self, thr_min, thr_max, blur=15, blob_params=None, detection_window=None):
    
        #GPIO.setmode(GPIO.BOARD)
        #GPIO.setup(IN3, GPIO.OUT, initial=GPIO.LOW)
        
        self.set_threshold(thr_min, thr_max)
        self.set_blur(blur)
        self.set_blob_params(blob_params)
        self.detection_window = detection_window
        
        self._t0 = time.time()
        
        self.blob_point = Point()
        self.blob_point.x = 0.0
        self.blob_point.y = 0.0
        self.blob_point.z = 0.0
    
        print (">> Publishing image to topic image_blob")
        self.image_pub = rospy.Publisher("/blob/image_blob",Image,queue_size=1)
        self.mask_pub = rospy.Publisher("/blob/image_mask",Image,queue_size=1)
        print (">> Publishing position to topic point_blob")
        self.blob_pub  = rospy.Publisher("/blob/point_blob",Point,queue_size=1)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/d400/color/image_raw",Image,self.callback)
        print ("<< Subscribed to topic /d400/color/image_raw")
        
    def set_threshold(self, thr_min, thr_max):
        self._threshold = [thr_min, thr_max]
        
    def set_blur(self, blur):
        self._blur = blur
      
    def set_blob_params(self, blob_params):
        self._blob_params = blob_params
        
    # def get_blob_relative_position(self, cv_image, keyPoint):
        # rows = float(cv_image.shape[0])
        # cols = float(cv_image.shape[1])
        # # print(rows, cols)
        # center_x    = 0.5*cols
        # center_y    = 0.5*rows
        # # print(center_x)
        # x = (keyPoint.pt[0] - center_x)/(center_x)
        # y = (keyPoint.pt[1] - center_y)/(center_y)
        # return(x,y)

###########################################
# COMPETITION CODE THAT YOU CNA GET RID OF LATER, DETECTS LED ON OR OFF

        
        
    def callback(self,data):
        #--- Assuming image is 320x240
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8") # change to cv_image
        except CvBridgeError as e:
            print(e)

        x = 1

        (rows,cols,channels) = cv_image.shape
        if cols > 60 and rows > 60 : 
            #--- Detect blobs
            keypoints, mask   = blob_detect(cv_image, self._threshold[0], self._threshold[1], self._blur,
                                            blob_params=self._blob_params, search_window=self.detection_window )
            #--- Draw search window and blobs
            cv_image    = blur_outside(cv_image, 10, self.detection_window)

            cv_image    = draw_window(cv_image, self.detection_window, line=1)
            cv_image    = draw_frame(cv_image)
            
            cv_image    = draw_keypoints(cv_image, keypoints) 
            
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
                self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask, "8UC1"))
            except CvBridgeError as e:
                print(e)            

            if keypoints == []:
                self.blob_point.x = 0
                self.blob_point.y = 0
                self.blob_point.z = 500

                self.blob_pub.publish(self.blob_point) 
            else:
                for i, keyPoint in enumerate(keypoints):
                    #--- Here you can implement some tracking algorithm to filter multiple detections
                    #--- We are simply getting the first result
                    x = keyPoint.pt[0]
                    y = keyPoint.pt[1]
                    s = keyPoint.size
                    print ("kp %d: s = %3d   x = %3d  y= %3d"%(i, s, x, y))
                    
                    #--- Find x and y position in camera adimensional frame
                    #x, y = get_blob_relative_position(cv_image, keyPoint)

                    #GPIO.output(IN3, GPIO.HIGH)
                    
                    self.blob_point.x = x
                    self.blob_point.y = y
                    self.blob_point.z = s

                    self.blob_pub.publish(self.blob_point) 
                    break
                    
            fps = 1.0/(time.time()-self._t0)
            self._t0 = time.time()
        elif x == 0:
            
            print('Original Dimensions : ',cv_image.shape)
            
            scale_percent = 50 # percent of original size
            width = int(cv_image.shape[1] * scale_percent / 100)
            height = int(cv_image.shape[0] * scale_percent / 100)
            dim = (width, height)
            # resize image
            resized = cv2.resize(cv_image, dim, interpolation = cv2.INTER_AREA)
            
            print('Resized Dimensions : ',resized.shape)
            
            # gray scale
            gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)

            #treshold

            retval, threshold = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

            contours = find_leds(threshold)

            print("Number of Contours found = " + str(len(contours)))

            cont_img = cv_image
            cv2.drawContours(cont_img, contours, -1, (0,255,0), 3)

            cv2.imshow('Contours', cont_img)

            #cv2.imshow("Resized image", threshold)

            display_img(resized, 1000)
            display_img(gray, 1000)
            display_img(threshold, 1000)


            #cv2.waitKey(0)
            #cv2.destroyAllWindows()


            
def main(args):
    #red_min = (10,135,75) # RED CHIP 10, 135, 75
    #red_max = (30, 255, 225) 
    red_min=(30, 4, 253) # RED CHIP 0, 121, 102
    red_max=(35, 15, 255) # RED CHIP 4, 255, 255
    yellow_min = (16,127,118)
    yellow_max = (42,255,255)
    green_min = (45,49,99)
    green_max = (86,255,255)
    pink_min = (166,135,97)
    pink_max = (255,255,255)
    white_min = (48,0,120)
    white_max = (119,80,221)
    # blue_mi0= (82,31,62)
    # blue_max = (106, 116, 193)     
    # blue_min = (55,40,0)
    # blue_max = (150, 255, 255)     
    
    blur     = 5
    min_size = 10
    max_size = 40
    
    #--- detection window respect to camera frame in [x_min, y_min, x_max, y_max] adimensional (0 to 1)
    x_min   = 0.0
    x_max   = 1.0
    y_min   = 0.0
    y_max   =1.0
    
    detection_window = [x_min, y_min, x_max, y_max]
    
    params = cv2.SimpleBlobDetector_Params()
         
    # Change thresholds
    params.minThreshold = 0;
    params.maxThreshold = 100;
     
    # Filter by Area.
    params.filterByArea = True
    params.minArea = 200
    params.maxArea = 20000
     
    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.1
    # params.maxCircularity = 0.6
     
    # Filter by Convexity
    params.filterByConvexity = False
    params.minConvexity = 0.2
     
    # Filter by Inertia
    params.filterByInertia = True
    params.minInertiaRatio = 0.2   

    rospy.init_node('blob_detector', anonymous=True)
    ic = BlobDetector(yellow_min, yellow_max, blur, params, detection_window)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
