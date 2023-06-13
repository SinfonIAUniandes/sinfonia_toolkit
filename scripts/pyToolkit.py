#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

import qi
import time
import rospy
import argparse
import sys
from robot_toolkit_msgs.srv import  show_image_srv
from std_srvs.srv import SetBool, Empty

class PyToolkit:

    def __init__(self, session):
        self.show_srv = rospy.Service('tablet/show_image', show_image_srv, 
                                      self.show_image_srv)
        self.hide_tablet_srv = rospy.Service('tablet/hide_tablet', show_image_srv, 
                                             self.hide_tablet_srv)
        self.show_webpage_srv = rospy.Service('tablet/show_webpage', show_image_srv, 
                                              self.show_webpage_srv)
        self.show_video_srv = rospy.Service('tablet/show_video', show_image_srv, 
                                            self.show_video_srv)
        self.autonomous_life_service = session.service("ALAutonomousLife")
        self.disable_autonomous_life_srv = rospy.Service('autonomous_life/disable', Empty, 
                                                         self.disable_autonomous_life_srv)
        self.ts = session.service("ALTabletService")
        self.posture_service = session.service("ALRobotPosture")
        self.awareness_service = session.service("ALBasicAwareness")

        self.disable_autonomous_life_srv = rospy.Service('autonomous_life/disable', Empty, self.disable_autonomous_life_srv)
        self.set_awareness_srv = rospy.Service('awareness', SetBool, self.set_awareness)


    def show_image_srv(self,req):
        url = req.url
        self.ts.showImage(url)
        time.sleep(5)
        return None

    def hide_tablet_srv(self,req):
        self.ts.hide()
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
    
    def disable_autonomous_life_srv(self,req):
        self.autonomous_life_service.setState("disabled")
        self.stand()
        print("[INFO]: Autonomous life is off")
    
    def stand(self):
        self.posture_service.goToPosture("Stand", 0.5)
        print("[INFO]: Robot is in default position")

    def set_awareness(self, req):
        """
        Turn on or off the basic awareness of the robot,
        e.g. looking for humans, self movements etc.

        :param state: If True set on, if False set off
        :type state: bool
        """
        if req.data:
            self.awareness_service.resumeAwareness()
            self.awareness_service.setEnabled(True)
            print("[INFO]: Awareness is turned on")
        else:
            self.awareness_service.pauseAwareness()
            self.awareness_service.setEnabled(False)
            print("[INFO]: Awareness is paused")

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
        print("Can't connect to Naoqi at ip \"" + args.ip + "\" on port" + str(args.port) + 
           ".\n""Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    pytoolkit = PyToolkit(session)
    rospy.init_node('pytoolkit')
    try:
        print(" --- pytoolkit successfully initialized ---")
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
