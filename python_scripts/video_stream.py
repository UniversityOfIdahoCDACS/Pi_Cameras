#!/usr/bin/env python
# video_stream.py
# Author: Nikolai Tiong
# Date: 3/1/21
# Version: 1.0
# 
# This python script streams video from a Raspberry Pi Camera. 
# It is used for demonstration and testing purposes.
#
# Assumes that the nodes are up and running on RosBox2 and on the Raspberry Pi
#
# format python video_stream.py -c <camera location> -n <node name>
import roslib
import sys
import rospy
import subprocess
import cv2
import PIL
import getopt
from std_msgs.msg import String
from sensor_msgs.msg import Image
from PIL import Image as Img_convert
from matplotlib import image as Img2
from cv_bridge import CvBridge, CvBridgeError
from numpy import asarray

def printInstructions():
  print ("Syntax:")
  print ("python video_stream.py -c <camera location> -n <node name>")
  print ("Camera must be specified e.g. front, above. Name defaults to \"test\".")


def main(argv):
    
    global camera 
    camera = ''
    name = ''
    try:
       opts, args = getopt.getopt(argv, "c:n:", ["camera", "name"])
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
      elif opt in ("-n", "--name"):
        name = arg
    if (camera == ''):
      print ("Camera not specified.")
      printInstructions()
      sys.exit(2) 
    if (name == ''):
      print ("Node name not specified. Using \"test\" as name.")
      name = 'test'
    video_stream = "/" + camera + "/raspicam_node/image_raw"
    topics = rospy.get_published_topics()
    length = len(topics)
    subTopicExists = False
    for i, j  in enumerate(topics):
      #print (topics[i][0])
      if topics[i][0] == video_stream:
        print("Publisher " + topics[i][0] + " exists on the node and can be subscribed to.")
        subTopicExists = True
    if (subTopicExists):
       print ("Subscribing to the topic " + video_stream + " with name \"" + name + "\"")
       cmd = "rosrun image_view image_view __name:=" + name + " image:=" + video_stream
       subprocess.call(cmd, shell=True)
       #ic = video_capture()
       
       try:
          rospy.spin()
       except KeyboardInterrupt:
          print("Shutting down")
       cv2.destroyAllWindows()
    else:
     print ("Topic " + video_stream + " doesn't exist.")  
if __name__ == '__main__':
    main(sys.argv[1:])
    
