#!/usr/bin/env python
# cam.py
# Author: Nikolai Tiong
# Date: 3-10-21
#
# This script runs on the raspberry pi's with cameras attached to them.
# It will start a subscriber node which will take a picture at full resolution 
# and then send it back over the <name>_image_capture publisher node
#
# It will also be running a raspicam_node via a launch file in catkin_ws/src/
# raspicam_node/launch/
# 
# The camera can be streamed by using the video_stream.py file located on the
# remote node.
# Full resolution (12MP) images can be captured by using the image_capture.py 
# file located on the remote node.
# A video stream cannot be active while an image is captured and will kill the
# stream temporarily while the image is taken. It will restart after the image 
# is sent. The end user will see the video pause temporarily (~10s).
#
# Both this file and the launch file will need to be modified with a new camera
# name so as to not clash with other existing camera nodes.
# Camera nodes as of 3-10-21:
#    front - the one on the front part of the cage facing the FANUC arm
#    above - the one on the track above the FANUC arm facing down
#    arm - the one attached to the end of the arm, currently the robot hand
#
# How to use:
#    python cam.py
#
################################################################################
# Change this to relevant name
camera = 'front'
################################################################################
# Change this to the relevant launch file located in catkin_ws/src/raspicam_node/launch/
# Current working launch files:
# camera_module_v2_1280x720_30fps.launch
# camera_module_v2_1920x1080_30fps.launch
video = "roslaunch raspicam_node camera_module_v2_1280x720_15fps.launch"
################################################################################
import roslib
import sys
import rospy
import picamera
import subprocess
import cv2
import PIL
import signal
import datetime
from std_msgs.msg import String
from sensor_msgs.msg import Image
from PIL import Image as Img_convert
from matplotlib import image as Img2
from cv_bridge import CvBridge, CvBridgeError
from numpy import asarray

class image_capture:
    def __init__(self, tcp_nodelay=True):
        self.image_pub = rospy.Publisher("/" + camera + "_image_capture", Image, queue_size = 1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/" + camera + "_image_command", String, self.callback)
        self.image_pub_status = rospy.Publisher("/" + camera + "_image_status", String, queue_size = 1)
        print(self.image_pub)
        print(self.image_sub)
    def callback(self, data):
        try:
            rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
            self.image_pub_status.publish("Message received by camera.")
            form = data.data
            killcmd = "rosnode kill /"+ camera + "/raspicam_node"
            subprocess.call(killcmd, shell=True)
            self.image_pub_status.publish("Camera Node Killed.")
            #rate = rospy.Rate(1)
            #rate.sleep()
            cmd = "raspistill -t 500 -e " + form + " -o test." + form
            subprocess.call(cmd, shell=True)
            self.image_pub_status.publish("Image Captured.")
            now = rospy.Time.now()
            date = datetime.datetime.fromtimestamp(now.secs)
            rospy.loginfo(date)
            rospy.loginfo("Image captured")

            capture = Img_convert.open('test.' + form)
            data = asarray(capture)
            data = cv2.putText(data, date.strftime('%m/%d/%Y %H:%M:%S'),  (50, 120),cv2.FONT_HERSHEY_SIMPLEX, 4, (255,255, 0), 5);
            self.image_pub_status.publish("Publishing Image.")
            rospy.loginfo("Publishing image")
            if form == "jpg" :
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(data, "bgr8"))
            elif form == "png" :
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(data, "bgra8"))
            elif form == "bmp" :
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(data, "bgr8"))
            rospy.loginfo("Published image")
            self.image_pub_status.publish("Image Published.")
            rospy.loginfo("Restarting video capture")
            p = subprocess.Popen(video, shell=True)
        except CvBridgeError as e:
            print(e)

def launch_camera():
    nodename = camera + '_image_capture'
    rospy.init_node(nodename, anonymous = True)
    p = subprocess.Popen(video, shell=True)
    print(p)

def main(args):
    ic = image_capture()
    launch_camera()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
    
