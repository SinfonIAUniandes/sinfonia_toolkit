#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

import qi
import time
import rospy
import argparse
import sys
from robot_toolkit_msgs.srv import  show_image_srv

class PyToolkit:

    def __init__(self, session):
        self.show_image_srv = rospy.Service('tablet/show_image', show_image_srv, self.show_image_srv)
        self.show_webpage_srv = rospy.Service('tablet/show_webpage', show_image_srv, self.show_webpage_srv)
        self.show_video_srv = rospy.Service('tablet/show_video', show_image_srv, self.show_video_srv)
        self.ts = session.service("ALTabletService")

    def show_image_srv(self,req):
        url = req.url
        self.ts.showImage(url)
        time.sleep(5)
        return None

    def show_webpage_srv(self,req):
        url = req.url
        self.ts.showWebview(url)
        time.sleep(5)
        return None

    def show_video_srv(self,req):
        url = req.url
        self.ts.playVideo(url)
        time.sleep(5)
        return None

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
			help="Robot IP address. On RObot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
			help="Naoqi port number")
    args = parser.parse_args()
    session = qi.Session()
    try:
        session.connect("tcp://" + args.ip + ":" + str(args.port))
    except RuntimeError:
        print("Can't connect to Naoqi at ip \"" + args.ip + "\" on port" + str(args.port) + ".\n""Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    pytoolkit = PyToolkit(session)
    rospy.init_node('pytoolkit')
    try:
        print(" --- pytoolkit successfully initialized ---")
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
