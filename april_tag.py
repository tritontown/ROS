#!/usr/bin/env python

import apriltag
import cv2
import numpy as np
import math
import rospy
#import pickle
#import picamera
import time
from math import atan2,degrees
from std_msgs.msg import Float32MultiArray as fl
from std_msgs.msg import MultiArrayLayout as MAL
from geometry_msgs.msg import Pose

def draw_tag(frame,center,corners):
    cv2.circle(frame,tuple(center.astype(int)),5,(0,0,255),3)
    cv2.circle(frame,tuple(corners[0].astype(int)),5,(0,0,255),3)
    cv2.circle(frame,tuple(corners[1].astype(int)),5,(0,255,0),3)
    cv2.circle(frame,tuple(corners[2].astype(int)),5,(255,0,0),3)
    cv2.circle(frame,tuple(corners[3].astype(int)),5,(0,0,0),3)
    mid = [np.int((corners[0][0]+corners[1][0])/2),np.int((corners[0][1]+corners[1][1])/2)]
    cv2.line(frame,tuple(center.astype(int)),tuple(mid),(0,0,255),5)
    angle = degrees(atan2(corners[1][1]-corners[0][1],corners[1][0]-corners[0][0]))
    #cv2.putText(frame,"angle: "+str(np.int(angle)),tuple(center.astype(int)),cv2.FONT_HERSHEY_SIMPLEX,2,128)
    return frame, angle

def main():
	video_capture = cv2.VideoCapture(0)
	video_capture.set(3, 640)
	video_capture.set(4, 480)

        # ROS Publisher
        pub = rospy.Publisher('tagPose', Pose, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(10)
    
	detector = apriltag.Detector(searchpath=apriltag._get_demo_searchpath())
	
        while (True):

            _, frame = video_capture.read()
                            
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            result = detector.detect(gray_frame)
                
            if result:
                center = result[0].center
                corners = result[0].corners
                frame, angle = draw_tag(frame,center,corners)
                
                tagPose = Pose()
                tagPose.position.x = center[0]
                tagPose.position.y = center[1]
                tagPose.position.z = 0

                tagPose.orientation.x = 0
                tagPose.orientation.y = 0
                tagPose.orientation.z = math.radians(angle)
                tagPose.orientation.w = 1

                print(tagPose)
                #rospy.loginfo(angle)
                pub.publish(tagPose)

            cv2.imshow("apriltags",frame)
            k = cv2.waitKey(1)

            if(k%256 == 32):
                break
        
    
        video_capture.release()


if __name__=="__main__":

    try:
        main()
    except rospy.ROSInterruptException:
        pass
