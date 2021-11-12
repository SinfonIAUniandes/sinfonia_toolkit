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


#ifndef SPEECH_SUBSCRIBER_HPP
#define SPEECH_SUBSCRIBER_HPP

#include "robot_toolkit/subscriber/base_subscriber.hpp"
#include <ros/ros.h>

#include "robot_toolkit_msgs/speech_msg.h"
#include "robot_toolkit_msgs/speech_parameters_msg.h"

namespace Sinfonia
{
    namespace Subscriber
    {
	class SpeechSubscriber: public BaseSubscriber<SpeechSubscriber>
	{
	    public:
		SpeechSubscriber( const std::string& name, const std::string& topic, const qi::SessionPtr& session );
		~SpeechSubscriber(){}

		void reset( ros::NodeHandle& nodeHandle );
		void speechCallback( const robot_toolkit_msgs::speech_msg& message );
		void shutdown();
		
		std::vector<float> setParameters(std::vector<float> parameters);
		std::vector<float> setDefaultParameters();
		std::vector<float> getParameters();
		
		void printSpeechParams(robot_toolkit_msgs::speech_parameters_msg parameters);
		robot_toolkit_msgs::speech_parameters_msg toSpeechParameters(std::vector<float> params);
		

	    private:
		std::string _speechTopic;
		qi::AnyObject _pTextToSpeech;
		qi::AnyObject _pTextToSpeechAnimated;
		ros::Subscriber _subscriberSpeech;
		std::string _language;
	}; 
    }
}
#endif