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



#ifndef CMD_VEL_SUBSCRIBER_HPP
#define CMD_VEL_SUBSCRIBER_HPP

#include "robot_toolkit/subscriber/base_subscriber.hpp"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeed.h>

namespace Sinfonia
{
    namespace Subscriber
    {

	class CmdVelSubscriber: public BaseSubscriber<CmdVelSubscriber>
	{
	    public:
		CmdVelSubscriber( const std::string& name, const std::string& cmdVelTopic, const qi::SessionPtr& session );
		~CmdVelSubscriber(){}

		void reset( ros::NodeHandle& nodeHandle );
		void cmdVelCallback( const geometry_msgs::TwistConstPtr& twistMsg );
		void shutdown();
		
		std::vector<float> getParameters();
		std::vector<float> setParameters(std::vector<float> parameters);
		std::vector<float> setDefaultParameters();
		
		

	    private:
		
		std::string _cmdVelTopic;

		qi::AnyObject _pMotion;
		ros::Subscriber _subscriberCmdVel;
		ros::Timer _timer;
		float _securityTime;
		void timerCallback(const ros::TimerEvent& event);
	}; 

    }
}
#endif
