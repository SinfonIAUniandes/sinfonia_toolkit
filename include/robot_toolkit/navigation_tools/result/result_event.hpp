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


#ifndef RESULT_EVENT_HPP
#define RESULT_EVENT_HPP

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/enable_shared_from_this.hpp>

#include <qi/session.hpp>

#include <ros/ros.h>

#include "robot_toolkit_msgs/audio_localization_msg.h"
#include <std_msgs/String.h>
#include "robot_toolkit/navigation_tools/result/result_publisher.hpp"

namespace Sinfonia
{
    namespace Navigation
    {
	class ResultEvent: public boost::enable_shared_from_this<ResultEvent>
	{
	    public:
		ResultEvent(std::string name, const float& frecuency, const qi::SessionPtr& session);
		~ResultEvent(){}
		
		void resetPublisher(ros::NodeHandle& nodeHandle);
		void shutdownPublisher();

		void startProcess();
		void stopProcess();
		void isPublishing(bool state){}

		bool isStarted(){}
		
		void setDefaultParameters(){}
		void setParameters(std::vector<int> parameters){}
		void shutdownEvents(){}
		void onResultCallback(std::string key,  qi::AnyValue value, std::string subscriberIdentifier);
		
	    private: 
		std::string _name;
		std::string _key;
		
		unsigned int _serviceId;
		
		boost::mutex _mutex;
		
		qi::SessionPtr _session;
		qi::AnyObject _pMemory;
		
		bool _isStarted;
		bool _isPublishing;
		
		boost::shared_ptr<Publisher::ResultPublisher > _publisher;
	};
	
	QI_REGISTER_OBJECT(ResultEvent, onResultCallback)
    }
}

#endif