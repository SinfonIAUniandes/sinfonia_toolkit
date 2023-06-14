#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

import qi
import time
import rospy
import argparse
import sys
from robot_toolkit_msgs.srv import tablet_service_srv, go_to_posture_srv, go_to_posture_srvResponse, tablet_service_srvResponse, go_to_posture_srvRequest
from std_srvs.srv import SetBool, SetBoolResponse, Empty
import ConsoleFormatter

class PyToolkit:
    # -----------------------------------------------------------------------------------------------------------------------
    # -----------------------------------------------------INIT--------------------------------------------------------------
    # -----------------------------------------------------------------------------------------------------------------------

    def __init__(self, session):
        # Service Naoqi Clients
        self.ALAutonomousLife = session.service("ALAutonomousLife")
        self.ALBasicAwareness = session.service("ALBasicAwareness")
        self.ALRobotPosture = session.service("ALRobotPosture")
        self.ALTabletService = session.service("ALTabletService")


        # Service ROS Servers - ALAutonomousLife
        self.autonomousSetStateServer = rospy.Service('pytoolkit/ALAutonomousLife/set_state_srv', SetBool, self.callback_autonomous_set_state_srv)
        print(consoleFormatter.format('ALAutonomousLife/set_state_srv on!', 'OKGREEN'))    


        # Service ROS Servers - ALBasicAwareness
        self.awarenessSetAwarenessServer = rospy.Service('pytoolkit/ALBasicAwareness/set_awareness_srv', SetBool, self.callback_awareness_set_awareness_srv)
        print(consoleFormatter.format('Set_awareness_srv on!', 'OKGREEN'))    

        
        # Service ROS Servers - ALRobotPosture
        self.postureGoToPostureServer = rospy.Service('pytoolkit/ALRobotPosture/go_to_posture_srv', go_to_posture_srv, self.callback_posture_go_to_posture_srv)
        

        # Service ROS Servers - ALTabletService
        self.tabletShowImageServer = rospy.Service('pytoolkit/ALTabletService/show_image_srv', tablet_service_srv, self.callback_tablet_show_image_srv)
        print(consoleFormatter.format('Show_image_srv on!', 'OKGREEN'))    

        self.tabletShowWebViewServer = rospy.Service('pytoolkit/ALTabletService/show_web_view_srv', tablet_service_srv, self.callback_tablet_show_web_view_srv)
        print(consoleFormatter.format('Show_web_view_srv on!', 'OKGREEN'))    

        self.tabletPlayVideoServer = rospy.Service('pytoolkit/ALTabletService/play_video_srv', tablet_service_srv, self.callback_tablet_play_video_srv)
        print(consoleFormatter.format('Play_video_srv on!', 'OKGREEN'))    

        self.tabletHideServer = rospy.Service('pytoolkit/ALTabletService/hide_srv', Empty, self.callback_tablet_hide_srv)
        print(consoleFormatter.format('Hide_srv on!', 'OKGREEN'))    


    # -----------------------------------------------------------------------------------------------------------------------
    # ----------------------------------------------------SERVICES CALLBACKS-------------------------------------------------
    # -----------------------------------------------------------------------------------------------------------------------
    
    # ----------------------------------------------------ALAutonomousLife------------------------------------------------

    def callback_autonomous_set_state_srv(self, req):
        print(consoleFormatter.format("\nRequested ALAutonomousLife/set_state_srv", "WARNING"))
        self.ALAutonomousLife.setAutonomousAbilityEnabled("All", req.data)
        if req.data:
            self.ALAutonomousLife.setState("interactive")
            print(consoleFormatter.format('Autonomous life is on!', 'OKGREEN'))
        else:
            self.ALAutonomousLife.setState("disabled")
            self.callback_posture_go_to_posture_srv(go_to_posture_srvRequest("stand"))
            print(consoleFormatter.format('Autonomous life is off!', 'OKGREEN'))
        return SetBoolResponse(True, "OK")

    # ----------------------------------------------------ALBasicAwareness------------------------------------------------

    def callback_awareness_set_awareness_srv(self, req):
        print(consoleFormatter.format("\nRequested ALBasicAwareness/set_awareness_srv", "WARNING"))
        if req.data:
            self.ALBasicAwareness.setEnabled(True)
            print(consoleFormatter.format('Awareness is on!', 'OKGREEN'))
        else:
            self.ALBasicAwareness.pauseAwareness()
            print(consoleFormatter.format('Awareness is off!', 'OKGREEN'))
        return SetBoolResponse(True, "OK")

    # ----------------------------------------------------ALRobotPosture------------------------------------------------
    
    def callback_posture_go_to_posture_srv(self, req):
        print(consoleFormatter.format("\nRequested ALRobotPosture/go_to_posture_srv", "WARNING"))
        if req.posture == "stand":
            self.ALRobotPosture.goToPosture("Stand", 0.5)
            print(consoleFormatter.format('Robot is in default position!', 'OKGREEN'))
        elif req.posture == "rest":
            self.ALRobotPosture.goToPosture("Crouch", 0.5)
            print(consoleFormatter.format('Robot is in rest position!', 'OKGREEN'))
        return go_to_posture_srvResponse("OK")

    # ----------------------------------------------------ALTabletService------------------------------------------------


    def callback_tablet_show_image_srv(self, req):
        print(consoleFormatter.format("\nRequested ALTabletService/show_image_srv", "WARNING"))
        self.ALTabletService.showImage(req.url)
        print(consoleFormatter.format('Image shown!', 'OKGREEN'))
        return tablet_service_srvResponse("OK")
    

    def callback_tablet_show_web_view_srv(self, req):
        print(consoleFormatter.format("\nRequested ALTabletService/show_web_view_srv", "WARNING"))
        self.ALTabletService.showWebview(req.url)
        print(consoleFormatter.format('Web view shown!', 'OKGREEN'))
        return tablet_service_srvResponse("OK")
    

    def callback_tablet_play_video_srv(self, req):
        print(consoleFormatter.format("\nRequested ALTabletService/play_video_srv", "WARNING"))
        self.ALTabletService.playVideo(req.url)
        print(consoleFormatter.format('Video played!', 'OKGREEN'))
        return tablet_service_srvResponse("OK")
    

    def callback_tablet_hide_srv(self, req):
        print(consoleFormatter.format("\nRequested ALTabletService/hide_srv", "WARNING"))
        self.ALTabletService.hide()
        print(consoleFormatter.format('Tablet hidden!', 'OKGREEN'))
        return


if __name__ == '__main__':
    consoleFormatter=ConsoleFormatter.ConsoleFormatter()
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
        print(consoleFormatter.format(" \n----------------------------------------------------------", "OKGREEN"))  
        print(consoleFormatter.format(" --------- PyToolkit node successfully initialized --------- ", "OKGREEN"))
        print(consoleFormatter.format(" ----------------------------------------------------------\n", "OKGREEN")) 
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
