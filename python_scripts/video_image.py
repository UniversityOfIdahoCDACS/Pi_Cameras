#!/usr/bin/env python
from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import datetime
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/arm/raspicam_node/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      now = rospy.Time.now()
      date = datetime.datetime.fromtimestamp(now.secs)
      cv_image = cv2.putText(cv_image, date.strftime('%m/%d/%Y %H:%M:%S'), (20,40),cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 1);
    #cv2.imshow("Image window", cv_image)
    #cv2.waitKey(3)
      cv2.imwrite("test.jpg", cv_image)
      print("Image saved as test.jpg")
      rospy.signal_shutdown("Image Saved")
    except CvBridgeError as e:
      print(e)

   

def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
