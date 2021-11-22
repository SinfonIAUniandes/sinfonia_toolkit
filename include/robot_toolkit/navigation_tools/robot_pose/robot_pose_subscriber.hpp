//======================================================================//
//  This software is free: you can redistribute it and/or modify        //
//  it under the terms of the GNU General Public License Version 3,     //
//  as published by the Free Software Foundation.                       //
//  This software is distributed in the hope that it will be useful,    //
//  but WITHOUT ANY WARRANTY; without even the implied warranty of      //
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE..  See the      //
//  GNU General Public License for more details.                        //
//  You should have received a copy of the GNU General Public License   //
//  Version 3 in the file COPYING that came with this distribution.     //
//  If not, see <http://www.gnu.org/licenses/>                          //
//======================================================================//
//                                                                      //
//      Copyright (c) 2019 SinfonIA Pepper RoboCup Team                 //
//      Sinfonia - Colombia                                             //
//      https://sinfoniateam.github.io/sinfonia/index.html              //
//                                                                      //
//======================================================================//

#ifndef ROBOT_POSE_SUBSCRIBER_HPP
#define ROBOT_POSE_SUBSCRIBER_HPP

#include "robot_toolkit/subscriber/base_subscriber.hpp"
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Pose2D.h>
#include "robot_toolkit_msgs/set_angles_msg.h"

namespace Sinfonia
{
    namespace Subscriber
    {
	class RobotPoseSubscriber: public BaseSubscriber<RobotPoseSubscriber>
	{
	    public:
		RobotPoseSubscriber( const std::string& name, const std::string& topic, const qi::SessionPtr& session );
		~RobotPoseSubscriber(){}

		void reset( ros::NodeHandle& nodeHandle );
		void robotPoseCallback( const geometry_msgs::Pose2D message );
		void shutdown();
		
		std::vector<float> getParameters(){}
		std::vector<float> setParameters(std::vector<float> parameters){}
		std::vector<float> setDefaultParameters(){}
		

	    private:
		std::string _topic;
		qi::AnyObject _pMemory;
		ros::Subscriber _subscriberRobotPose;
	}; 
    }
}
#endif