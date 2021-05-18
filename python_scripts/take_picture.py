#! /urs/bin/env python
# take_picture.py
# Author: Nikolai Tiong
# Date: 3/1/21
# Version: 1.0
# 
# This python script takes a full resolution picture using one of the Raspberry 
# Pi Cameras. It is used for demonstration and testing purposes.
#
# Assumes that the nodes are up and running on RosBox2 and on the Raspberry Pi
#
# format python take_picture.py -c <camera location> -f <image type> -n <name>
import roslib
import sys
import rospy
import cv2
import getopt
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from numpy import asarray
from PIL import Image as Img2

bridge = CvBridge()

def printInstructions():
  print ("Syntax:")
  print ("python take_picture.py -c <camera location> -f <image format> -n <filename>")
  print ("Camera must be specified e.g. front, above. Image format defaults to \"jpg\" and name defaults to \"test\".")

def main(argv):
  camera =''
  form = ''
  name = ''
  if (len(argv) > 6):
     print ("Too many arguments specified.")
     printInstructions()
     sys.exit(2)
  try:
     opts, args = getopt.getopt(argv, "c:f:n:", ["camera", "format", "name"])
  except getopt.GetoptError:
     print("Invalid option specified")
     printInstructions()
     sys.exit(2)
     
  for opt, arg in opts:
     if opt in ("-c", "--camera"):
        camera = arg
        if (not camera.find("-")):
           print("\"-\" not allowed in camera name.")
           printInstructions()
           sys.exit(2)
     elif opt in ("-f", "--format"):
        if (arg == "png" or arg == "jpg" or arg == "bmp"):
           form = arg
        else:
           print ("Format type is not png, jpg or bmp")
           printInstructions()
           sys.exit(2)
     elif opt in ("-n", "--name"):
        name = arg
     else:
        print ("Invalid option specified.")
        printInstructions()
        sys.exit(2)
  if (camera == ''):
     print ("Camera not specified.")
     printInstructions()
     sys.exit(2) 
  if (form == ''):
     print ("Image format not specified. Using \"jpg\".")
     form = 'jpg'
  if (name == ''):
     print ("File name not specified. Using \"test\" as name.")
     name = 'test'
  publisher = "/" + camera + "_image_command"
  pub = rospy.Publisher(publisher, String, queue_size = 10)
  pub_node = camera + '_image_command'
  reply = '/' + camera + '_image_capture'
  #print (reply)
  topics = rospy.get_published_topics()
  length = len(topics)
  subTopicExists = False
  for i, j  in enumerate(topics):
     #print (topics[i][0])
     if topics[i][0] == reply:
        print("Publisher " + topics[i][0] + " exists on the node and can be subscribed to.")
        subTopicExists = True
  if (subTopicExists):
     rospy.init_node(pub_node)
     print ("Initialized publisher " + publisher)
     rate = rospy.Rate(1)
     rate.sleep()
     msg = String()
     msg.data= form
     pub.publish(msg)
     print("Publishing message \"" + form + "\" to topic " + publisher)
     rospy.loginfo(msg)
     print("Subscribing to topic " + reply)
     img = rospy.wait_for_message(reply, Image)
     print("Received image")
     cv2_img = bridge.imgmsg_to_cv2(img, "rgb8")
     cv2.imwrite(name + "." + form, cv2_img)
     print("Image saved as " + name + "." + form)
     rospy.signal_shutdown("Image Saved")
  else:
     print ("Topic " + reply + " doesn't exist.")


if __name__ == '__main__':
    main(sys.argv[1:])
