#!/usr/bin/env python
import roslib
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 
from classifier_msgs.msg import Classifier
from classifier_msgs.msg import ClassifierArray

  
def get_coordinate(event, x, y, flags, param):
    global mouseX, mouseY
    if event == cv2.EVENT_LBUTTONDBLCLK:
        mouseX,mouseY = x,y
#         print "Pixel values are %s,%s" % (mouseX, mouseY) 
#         yaw = atan2((320 - mouseX), 514.681661)
#         vectorX = cos(yaw) 
#         vectorY = sin(yaw) 
#         print " yaw:       %s" % (yaw *180/pi)       
#         print " vectorX %s" % (vectorX)
#         print " vectorY %s" % (vectorY)
        classifierPub = rospy.Publisher('Classifier_Data', ClassifierArray, queue_size=10)
        message = Classifier()
        message.header.stamp = rospy.get_rostime()
        classifierData = ClassifierArray()
        message.strength = 2.0
        message.point.x = mouseX
        message.point.y = mouseY
        message.prediction = 1.0
        message.type = "Human Controller"
        message.red = 1.0
        message.green = 0.0 
        message.blue = 0.0
        classifierData.classifiers.append(message)
        classifierPub.publish(classifierData)
           
         
       
def image_callback(msg):

    try:
        cv2_img = CvBridge().imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    cv2.namedWindow('Image Window')
    cv2.setMouseCallback('Image Window',get_coordinate)    
    cv2.imshow('Image Window',cv2_img)
    cv2.waitKey(100)    
        

def main():
    rospy.init_node('image_listener')
    rospy.Subscriber("/GETbot/xtion/rgb/image_raw", Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
