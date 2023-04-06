#!/usr/bin/env python

import rospy
import cv2
import cv_bridge
import np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher("/cmd_vel",Twist, queue_size=1)
    self.twist = Twist()
  
  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 
    # Color to follow
    sensitivity = 80
    lower_white = np.array([0,0,255-sensitivity])
    upper_white = np.array([255,sensitivity,255])
    mask = cv2.inRange(hsv, lower_white, upper_white)
    
    # Forcing view to be restricted to avoid disturbances
    h, w, d = image.shape
    #search_top = 3*h/4
    #search_bot = 3*h/4 + 20
    #mask[0:search_top, 0:w] = 0
    #mask[search_bot:h, 0:w] = 0

    # calculate moments of binary image
    M = cv2.moments(mask)

    if M['m00'] > 0:
      # calculate x,y coordinate of center
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])

      #Highlight the center
      #cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

      # CONTROL starts
      err = cx - w/2
      self.twist.linear.x = 0.2
      self.twist.angular.z = -float(err) / 100
      self.cmd_vel_pub.publish(self.twist)
      # CONTROL ends

    cv2.imshow("mask",mask)
    #cv2.imshow("output", image)
    cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
