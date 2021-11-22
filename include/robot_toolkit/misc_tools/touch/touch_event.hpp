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


#ifndef TOUCH_EVENT_REGISTER_HPP
#define TOUCH_EVENT_REGISTER_HPP

#include <qi/session.hpp>

#include "robot_toolkit/misc_tools/touch/touch_publisher.hpp"
#include <boost/enable_shared_from_this.hpp>

namespace Sinfonia
{
   namespace MiscToolsEvents
    {
	class TouchEvent: public boost::enable_shared_from_this<TouchEvent>
	{
	    public:
		TouchEvent(std::string name, const float& frecuency, const qi::SessionPtr& session);
		~TouchEvent(){}
		
		void resetPublisher(ros::NodeHandle& nodeHandle);
		void shutdownPublisher();

		void startProcess();
		void stopProcess();
		void isPublishing(bool state){}

		bool isStarted(){}
		
		void setDefaultParameters(){}
		void setParameters(std::vector<int> parameters){}
		void shutdownEvents(){}
		
		
		
		void touchCallback(std::string key,  qi::AnyValue value, std::string subscriberIdentifier);
		
	    private: 
		std::string _name;
		std::string _keyRightBumperPressed;
		std::string _keyLeftBumperPressed;
		std::string _keyBackBumperPressed;
		std::string _keyFrontTactilTouched;
		std::string _keyMiddleTactilTouched;
		std::string _keyRearTactilTouched;
		std::string _keyHandRightBackTouched;
		std::string _keyHandRightLeftTouched;
		std::string _keyHandRightRightTouched;
		std::string _keyHandLeftBackTouched;
		std::string _keyHandLeftLeftTouched;
		std::string _keyHandLeftRightTouched;
		
		unsigned int _serviceId; 
		
		unsigned int _serviceIdRightBumper;
		unsigned int _serviceIdLeftBumperPressed;
		unsigned int _serviceIdBackBumperPressed;
		unsigned int _serviceIdFrontTactil;
		unsigned int _serviceIdMiddleTactil;
		unsigned int _serviceIdRearTactil;
		unsigned int _serviceIdHandRightBack;
		unsigned int _serviceIdHandRightLeft;
		unsigned int _serviceIdHandRightRight;
		unsigned int  _serviceIdHandLeftBack;
		unsigned int _serviceIdHandLeftLeft;
		unsigned int  _serviceIdHandLeftRight;
		
		
		
		boost::mutex _mutex;
		
		qi::SessionPtr _session;
		qi::AnyObject _pMemory;
		
		bool _isStarted;
		bool _isPublishing;
		
		boost::shared_ptr<Publisher::TouchPublisher> _publisher;
	};
    
    QI_REGISTER_OBJECT(TouchEvent, touchCallback)
    }
}

#endif