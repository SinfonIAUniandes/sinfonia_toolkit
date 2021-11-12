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

#include "robot_toolkit/navigation_tools/result/result_event.hpp"

namespace Sinfonia
{
    namespace Navigation
    {
	ResultEvent::ResultEvent(std::string name, const float& frecuency, const qi::SessionPtr& session)
	{
	    _session = session;
	    _pMemory = session->service("ALMemory");
	    _key = "NAOqiPlanner/Result";
	    _name = name;
	    _serviceId = 0;
	    _isStarted = false;
	    _publisher = boost::make_shared<Publisher::ResultPublisher>("/navigation/result");
	}

	void ResultEvent::resetPublisher(ros::NodeHandle& nodeHandle)
	{
	    _publisher->reset(nodeHandle);
	}

	void ResultEvent::shutdownPublisher()
	{
	    _publisher->shutdown();
	}
	
	void ResultEvent::startProcess()
	{
	    boost::mutex::scoped_lock startLock(_mutex);
	    if( !_isStarted )
	    {
		if( !_serviceId )
		{
		    std::string serviceName = std::string("ROS-Driver") + _key;
		    _serviceId = _session->registerService(serviceName, this->shared_from_this());
		    _pMemory.call<void>("subscribeToEvent", _key.c_str(), serviceName, "onResultCallback");		
		}
		_isStarted = true;
	    }
	}
	
	void ResultEvent::stopProcess()
	{
	    boost::mutex::scoped_lock stopLock(_mutex);
	    std::string serviceName = std::string("ROS-Driver") + _key;
	    if(_isStarted)
	    {
		_pMemory.call<void>("unsubscribeToEvent", _key.c_str(), serviceName);
		if(_serviceId)
		{
		    _session->unregisterService(_serviceId);
		    _serviceId = 0;
		}
		_isStarted = false;
	    }
	}
	
	void ResultEvent::onResultCallback(std::string key, qi::AnyValue value, std::string subscriberIdentifier)
	{
	    static const boost::shared_ptr<std_msgs::String> message = boost::make_shared<std_msgs::String>();
	    message->data = value.asString();
	    _publisher->publish(message);
	}
    }
}
