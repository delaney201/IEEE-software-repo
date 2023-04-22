#!/usr/bin/python
"""
Gets the position of the blob and it commands to steer the wheels

Subscribes to 
    /blob/point_blob
    
Publishes commands to 
    /dkcar/control/cmd_vel    

"""
import math, time
import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import Int8, Float32
from geometry_msgs.msg import Point

IN1 = 33
IN2 = 35
IN3 = 37

K_LAT_DIST_TO_STEER     = 2.0

def saturate(value, min, max):
    if value <= min: return(min)
    elif value >= max: return(max)
    else: return(value)

class ChaseBall():
    def __init__(self):

        # Initialize to 000, no movement
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(IN1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(IN2, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(IN3, GPIO.OUT, initial=GPIO.LOW)
        
        self.blob_x         = 0.0
        self.blob_y         = 0.0
        self.blob_s         = 0.0
        self._time_detected = 0.0
        self.direction      = 0
        
        self.sub_center = rospy.Subscriber("/blob/point_blob", Point, self.update_ball)
        rospy.loginfo("Subscribers set")
        
        self.pub_dir = rospy.Publisher("/dkcar/control/direction", Int8, queue_size=5)
        rospy.loginfo("Publisher set")
        
        
        self._time_steer        = 0
        self._steer_sign_prev   = 0
        
    @property
    def is_detected(self): return(time.time() - self._time_detected < 1.0)
        
    def update_ball(self, message):
        self.blob_x = message.x
        self.blob_y = message.y
        self.blob_s = message.z
        self._time_detected = time.time()

        self.get_control_action()
        # rospy.loginfo("Ball detected: %.1f  %.1f "%(self.blob_x, self.blob_y))

    def get_control_action(self):
        """
        Based on the current ranges, calculate the command
        
        Steer will be added to the commanded throttle
        throttle will be multiplied by the commanded throttle
        """
        steer_action    = 0.0
        throttle_action = 0.0
        
        #if self.is_detected:
            #--- Apply steering, proportional to how close is the object
        if self.blob_s == 500:
            # Nothing detected, Don't move 000
            print("Nothin detected")
            self.direction = 0
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN2, GPIO.LOW)
            #GPIO.output(IN3, GPIO.LOW)
        else:
            #print("s value: ", self.blob_s)
            #print("x value: ", self.blob_x)
            #print("y value: ", self.blob_y)
            self.direction = 1
            if self.blob_x > 180:
            # Rotate Right
                #print("Not centered")
                print("Rotate Right")
                GPIO.output(IN1, GPIO.HIGH) 
                GPIO.output(IN2, GPIO.HIGH) # i2 in arduino
                GPIO.output(IN3, GPIO.LOW)
            elif self.blob_x < 140:
                print("Rotate Left")
                GPIO.output(IN1, GPIO.LOW) # i1 in arduino
                GPIO.output(IN2, GPIO.HIGH)
                GPIO.output(IN3, GPIO.LOW)
            else:
                print("Forward")
                GPIO.output(IN1, GPIO.HIGH)
                GPIO.output(IN2, GPIO.LOW)
                GPIO.output(IN3, GPIO.HIGH)
                if self.blob_s >= 80:
                    x = 800
                    while x != 0:
                        print("Continue")
                        GPIO.output(IN1, GPIO.HIGH)
                        GPIO.output(IN2, GPIO.LOW)
                        GPIO.output(IN3, GPIO.HIGH) # i0 in arduino
                        x = x-1
                    # Move the claw
                    #GPIO.output(IN1, GPIO.LOW) # i1 in the arduino
                    #GPIO.output(IN2, GPIO.HIGH) # i2 in arduino
                    #GPIO.output(IN3, GPIO.HIGH) # i0 in arduino
                    




"""
            # Check for the rotation
            if self.blob_s == 16 or self.blob_s == 17:
                if self.blob_x > 151:
                    # Rotate Right
                    GPIO.output(IN1, GPIO.HIGH)
                    GPIO.output(IN2, GPIO.LOW)
                    GPIO.output(IN3, GPIO.LOW)
                else:
                    # Move Forawrd
                    GPIO.output(IN1, GPIO.LOW)
                    GPIO.output(IN2, GPIO.LOW)
                    GPIO.output(IN3, GPIO.HIGH)
            if self.blob_s >= 18 and self.blob_s <= 22:
                if self.blob_x > 161:
                    # Rotate Right
                    GPIO.output(IN1, GPIO.HIGH)
                    GPIO.output(IN2, GPIO.LOW)
                    GPIO.output(IN3, GPIO.LOW)
                else:
                    # Move Forawrd
                    GPIO.output(IN1, GPIO.LOW)
                    GPIO.output(IN2, GPIO.LOW)
                    GPIO.output(IN3, GPIO.HIGH)
            if self.blob_s >= 23 and self.blob_s <= 28:
                if self.blob_x > 168:
                    # Rotate Right
                    GPIO.output(IN1, GPIO.HIGH)
                    GPIO.output(IN2, GPIO.LOW)
                    GPIO.output(IN3, GPIO.LOW)
                else:
                    # Move Forawrd
                    GPIO.output(IN1, GPIO.LOW)
                    GPIO.output(IN2, GPIO.LOW)
                    GPIO.output(IN3, GPIO.HIGH)
            if self.blob_s >= 29 and self.blob_s <= 34:
                if self.blob_x > 176:
                    # Rotate Right
                    GPIO.output(IN1, GPIO.HIGH)
                    GPIO.output(IN2, GPIO.LOW)
                    GPIO.output(IN3, GPIO.LOW)
                else:
                    # Move Forawrd
                    GPIO.output(IN1, GPIO.LOW)
                    GPIO.output(IN2, GPIO.LOW)
                    GPIO.output(IN3, GPIO.HIGH)
            if self.blob_s >= 35 and self.blob_s <= 40:
                if self.blob_x > 188:
                    # Rotate Right
                    GPIO.output(IN1, GPIO.HIGH)
                    GPIO.output(IN2, GPIO.LOW)
                    GPIO.output(IN3, GPIO.LOW)
                else:
                    # Move Forawrd
                    GPIO.output(IN1, GPIO.LOW)
                    GPIO.output(IN2, GPIO.LOW)
                    GPIO.output(IN3, GPIO.HIGH)
            if self.blob_s >= 41 and self.blob_s <= 45:
                if self.blob_x > 190:
                    # Rotate Right
                    GPIO.output(IN1, GPIO.HIGH)
                    GPIO.output(IN2, GPIO.LOW)
                    GPIO.output(IN3, GPIO.LOW)
                else:
                    # Move Forawrd
                    GPIO.output(IN1, GPIO.LOW)
                    GPIO.output(IN2, GPIO.LOW)
                    GPIO.output(IN3, GPIO.HIGH)
            if self.blob_s >= 46 and self.blob_s <= 50:
                if self.blob_x > 193:
                    # Rotate Right
                    GPIO.output(IN1, GPIO.HIGH)
                    GPIO.output(IN2, GPIO.LOW)
                    GPIO.output(IN3, GPIO.LOW)
                else:
                    # Move Forawrd
                    GPIO.output(IN1, GPIO.LOW)
                    GPIO.output(IN2, GPIO.LOW)
                    GPIO.output(IN3, GPIO.HIGH)
            if self.blob_s >= 51 and self.blob_s <= 56:
                if self.blob_x > 197:
                    # Rotate Right
                    GPIO.output(IN1, GPIO.HIGH)
                    GPIO.output(IN2, GPIO.LOW)
                    GPIO.output(IN3, GPIO.LOW)
                else:
                    # Move Forawrd
                    GPIO.output(IN1, GPIO.LOW)
                    GPIO.output(IN2, GPIO.LOW)
                    GPIO.output(IN3, GPIO.HIGH)
            if self.blob_s >= 56 and self.blob_s <= 60:
                if self.blob_x > 202:
                    # Rotate Right
                    GPIO.output(IN1, GPIO.HIGH)
                    GPIO.output(IN2, GPIO.LOW)
                    GPIO.output(IN3, GPIO.LOW)
                else:
                    # Move Forawrd
                    GPIO.output(IN1, GPIO.LOW)
                    GPIO.output(IN2, GPIO.LOW)
                    GPIO.output(IN3, GPIO.HIGH)
            if self.blob_s >= 61 and self.blob_s <= 65:
                if self.blob_x > 210:
                    # Rotate Right
                    GPIO.output(IN1, GPIO.HIGH)
                    GPIO.output(IN2, GPIO.LOW)
                    GPIO.output(IN3, GPIO.LOW)
                else:
                    # Move Forawrd
                    GPIO.output(IN1, GPIO.LOW)
                    GPIO.output(IN2, GPIO.LOW)
                    GPIO.output(IN3, GPIO.HIGH)
            if self.blob_s >= 66 and self.blob_s <= 70:
                if self.blob_x > 215:
                    # Rotate Right
                    GPIO.output(IN1, GPIO.HIGH)
                    GPIO.output(IN2, GPIO.LOW)
                    GPIO.output(IN3, GPIO.LOW)
                else:
                    # Move Forawrd
                    GPIO.output(IN1, GPIO.LOW)
                    GPIO.output(IN2, GPIO.LOW)
                    GPIO.output(IN3, GPIO.HIGH)
            if self.blob_s >= 71 and self.blob_s <= 75:
                if self.blob_x > 222:
                    # Rotate Right
                    GPIO.output(IN1, GPIO.HIGH)
                    GPIO.output(IN2, GPIO.LOW)
                    GPIO.output(IN3, GPIO.LOW)
                else:
                    # Move Forawrd
                    GPIO.output(IN1, GPIO.LOW)
                    GPIO.output(IN2, GPIO.LOW)
                    GPIO.output(IN3, GPIO.HIGH)
            if self.blob_s >= 76 and self.blob_s <= 80:
                if self.blob_x > 226:
                    # Rotate Right
                    GPIO.output(IN1, GPIO.HIGH)
                    GPIO.output(IN2, GPIO.LOW)
                    GPIO.output(IN3, GPIO.LOW)
                else:
                    # Move Forawrd
                    GPIO.output(IN1, GPIO.LOW)
                    GPIO.output(IN2, GPIO.LOW)
                    GPIO.output(IN3, GPIO.HIGH)
            """

          
            
if __name__ == "__main__":

    rospy.init_node('chase_ball')
    
    chase_ball     = ChaseBall()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")         