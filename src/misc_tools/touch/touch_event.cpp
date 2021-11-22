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


#include "robot_toolkit/misc_tools/touch/touch_event.hpp"


namespace Sinfonia 
{
    namespace MiscToolsEvents
    {
	TouchEvent::TouchEvent(std::string name, const float& frecuency, const qi::SessionPtr& session)
	{
	    _session = session;
	    _pMemory = session->service("ALMemory");
	    _name = name;
	    _serviceIdRightBumper = 0;
	    _serviceIdLeftBumperPressed = 0;
	    _serviceIdBackBumperPressed = 0;
	    _serviceIdFrontTactil = 0;
	    _serviceIdMiddleTactil = 0;
	    _serviceIdRearTactil = 0;
	    _serviceIdHandRightBack = 0;
	    _serviceIdHandRightLeft = 0;
	    _serviceIdHandRightRight = 0;
	    _serviceIdHandLeftBack = 0;
	    _serviceIdHandLeftLeft = 0;
	    _serviceIdHandLeftRight = 0;
	    
	    std::string _name;
	    _keyRightBumperPressed = "RightBumperPressed";
	    _keyLeftBumperPressed = "LeftBumperPressed";
	    _keyBackBumperPressed = "BackBumperPressed";
	    _keyFrontTactilTouched = "FrontTactilTouched";
	    _keyMiddleTactilTouched = "MiddleTactilTouched";
	    _keyRearTactilTouched = "RearTactilTouched";
	    _keyHandRightBackTouched = "HandRightBackTouched";
	    _keyHandRightLeftTouched = "HandRightLeftTouched";
	    _keyHandRightRightTouched = "HandRightRightTouched";
	    _keyHandLeftBackTouched = "HandLeftBackTouched";
	    _keyHandLeftLeftTouched = "HandLeftLeftTouched";
	    _keyHandLeftRightTouched = "HandLeftRightTouched";
	    
	    _isStarted = false;
	    _publisher = boost::make_shared<Publisher::TouchPublisher>("/touch");
	}

	void TouchEvent::startProcess()
	{
	    boost::mutex::scoped_lock startLock(_mutex);
	    if( !_isStarted )
	    {
		if( !_serviceIdRightBumper )
		{
		    std::string serviceName = std::string("ROS-Driver") + _keyRightBumperPressed;
		    _serviceIdRightBumper = _session->registerService(serviceName, this->shared_from_this());
		    _pMemory.call<void>("subscribeToEvent",_keyRightBumperPressed.c_str(), serviceName, "touchCallback");		
		}
		if( !_serviceIdLeftBumperPressed )
		{
		    std::string serviceName = std::string("ROS-Driver") + _keyLeftBumperPressed;
		    _serviceIdLeftBumperPressed = _session->registerService(serviceName, this->shared_from_this());
		    _pMemory.call<void>("subscribeToEvent",_keyLeftBumperPressed.c_str(), serviceName, "touchCallback");		
		}
		if( !_serviceIdBackBumperPressed )
		{
		    std::string serviceName = std::string("ROS-Driver") + _keyBackBumperPressed;
		    _serviceIdBackBumperPressed = _session->registerService(serviceName, this->shared_from_this());
		    _pMemory.call<void>("subscribeToEvent",_keyBackBumperPressed.c_str(), serviceName, "touchCallback");		
		}
		if( !_serviceIdFrontTactil )
		{
		    std::string serviceName = std::string("ROS-Driver") + _keyFrontTactilTouched;
		    _serviceIdFrontTactil = _session->registerService(serviceName, this->shared_from_this());
		    _pMemory.call<void>("subscribeToEvent",_keyFrontTactilTouched.c_str(), serviceName, "touchCallback");		
		}
		if( !_serviceIdMiddleTactil )
		{
		    std::string serviceName = std::string("ROS-Driver") + _keyMiddleTactilTouched;
		    _serviceIdMiddleTactil = _session->registerService(serviceName, this->shared_from_this());
		    _pMemory.call<void>("subscribeToEvent",_keyMiddleTactilTouched.c_str(), serviceName, "touchCallback");		
		}
		if( !_serviceIdRearTactil )
		{
		    std::string serviceName = std::string("ROS-Driver") + _keyRearTactilTouched;
		    _serviceIdRearTactil = _session->registerService(serviceName, this->shared_from_this());
		    _pMemory.call<void>("subscribeToEvent",_keyRearTactilTouched.c_str(), serviceName, "touchCallback");		
		}
		if( !_serviceIdHandRightBack )
		{
		    std::string serviceName = std::string("ROS-Driver") + _keyHandRightBackTouched;
		    _serviceIdHandRightBack = _session->registerService(serviceName, this->shared_from_this());
		    _pMemory.call<void>("subscribeToEvent",_keyHandRightBackTouched.c_str(), serviceName, "touchCallback");		
		}
		if( !_serviceIdHandRightLeft )
		{
		    std::string serviceName = std::string("ROS-Driver") + _keyHandRightLeftTouched;
		    _serviceIdHandRightLeft = _session->registerService(serviceName, this->shared_from_this());
		    _pMemory.call<void>("subscribeToEvent",_keyHandRightLeftTouched.c_str(), serviceName, "touchCallback");		
		}
		if( !_serviceIdHandRightRight )
		{
		    std::string serviceName = std::string("ROS-Driver") + _keyHandRightRightTouched;
		    _serviceIdHandRightRight = _session->registerService(serviceName, this->shared_from_this());
		    _pMemory.call<void>("subscribeToEvent",_keyHandRightRightTouched.c_str(), serviceName, "touchCallback");		
		}
		if( !_serviceIdHandLeftBack )
		{
		    std::string serviceName = std::string("ROS-Driver") + _keyHandLeftBackTouched;
		    _serviceIdHandLeftBack = _session->registerService(serviceName, this->shared_from_this());
		    _pMemory.call<void>("subscribeToEvent",_keyHandLeftBackTouched.c_str(), serviceName, "touchCallback");		
		}
		if( !_serviceIdHandLeftLeft )
		{
		    std::string serviceName = std::string("ROS-Driver") + _keyHandLeftLeftTouched;
		    _serviceIdHandLeftLeft = _session->registerService(serviceName, this->shared_from_this());
		    _pMemory.call<void>("subscribeToEvent",_keyHandLeftLeftTouched.c_str(), serviceName, "touchCallback");		
		}
		if( !_serviceIdHandLeftRight )
		{
		    std::string serviceName = std::string("ROS-Driver") + _keyHandLeftRightTouched;
		    _serviceIdHandLeftRight = _session->registerService(serviceName, this->shared_from_this());
		    _pMemory.call<void>("subscribeToEvent",_keyHandLeftRightTouched.c_str(), serviceName, "touchCallback");		
		}
		_isStarted = true;
	    }
	}
	void TouchEvent::stopProcess()
	{
	    boost::mutex::scoped_lock stopLock(_mutex);
	    if(_isStarted)
	    {
		if(_serviceIdRightBumper )
		{
		    _session->unregisterService(_serviceIdRightBumper);
		    _serviceIdRightBumper = 0;
		}
		if(_serviceIdLeftBumperPressed )
		{
		    _session->unregisterService(_serviceIdLeftBumperPressed);
		    _serviceIdLeftBumperPressed = 0;		
		}
		if( !_serviceIdBackBumperPressed )
		{
		    _session->unregisterService(_serviceIdBackBumperPressed);
		    _serviceIdBackBumperPressed = 0;		
		}
		if( !_serviceIdFrontTactil )
		{
		   _session->unregisterService(_serviceIdFrontTactil);
		    _serviceIdFrontTactil = 0;		
		}
		if( !_serviceIdMiddleTactil )
		{
		   _session->unregisterService(_serviceIdMiddleTactil);
		    _serviceIdMiddleTactil = 0;	
		}
		if( !_serviceIdRearTactil )
		{
		   _session->unregisterService(_serviceIdRearTactil);
		    _serviceIdRearTactil = 0;	
		}
		if( !_serviceIdHandRightBack )
		{
		    _session->unregisterService(_serviceIdHandRightBack);
		    _serviceIdHandRightBack = 0;		
		}
		if( !_serviceIdHandRightLeft )
		{
		   _session->unregisterService(_serviceIdHandRightLeft);
		    _serviceIdHandRightLeft = 0;	
		}
		if( !_serviceIdHandRightRight )
		{
		   _session->unregisterService(_serviceIdHandRightRight);
		    _serviceIdHandRightRight = 0;		
		}
		if( !_serviceIdHandLeftBack )
		{
		    _session->unregisterService(_serviceIdHandLeftBack);
		    _serviceIdHandLeftBack = 0;	
		}
		if( !_serviceIdHandLeftLeft )
		{
		   _session->unregisterService(_serviceIdHandLeftLeft);
		    _serviceIdHandLeftLeft = 0;		
		}
		if( !_serviceIdHandLeftRight )
		{
		   _session->unregisterService(_serviceIdHandLeftRight);
		    _serviceIdHandLeftRight = 0;		
		}
		_isStarted = false;
	    }
	}
	
	void TouchEvent::touchCallback(std::string key, qi::AnyValue value, std::string subscriberIdentifier)
	{
	    robot_toolkit_msgs::touch_msg message;
	    
	    if(key == "RightBumperPressed")
	    {
		message.name = "bumper_right";
	    }
	    else if(key == "LeftBumperPressed")
	    {
		message.name = "bumber_left";
	    }
	    else if(key == "BackBumperPressed")
	    {
		message.name = "bumber_back";
	    }
	    else if(key == "FrontTactilTouched")
	    {
		message.name = "head_front";
	    }
	    else if(key == "MiddleTactilTouched")
	    {
		message.name = "head_middle";
	    }
	    else if(key == "RearTactilTouched")
	    {
		message.name = "head_rear";
	    }
	    else if(key == "HandRightBackTouched")
	    {
		message.name = "hand_right_back";
	    }
	    else if(key == "HandRightLeftTouched")
	    {
		message.name = "hand_right_left";
	    }
	    else if(key == "HandRightRightTouched")
	    {
		message.name = "hand_right_right";
	    }
	    else if(key == "HandLeftBackTouched")
	    {
		message.name = "hand_left_back";
	    }
	    else if(key == "HandLeftLeftTouched")
	    {
		message.name = "hand_left_left";
	    }
	    else if(key == "HandLeftRightTouched")
	    {
		message.name = "hand_left_right";
	    }
	    message.state = (bool)value.toFloat();
	    std::cout << "key -> " << key << std::endl;
	    std::cout << "value -> " << value.toFloat() << std::endl;
	    _publisher->publish(message);
	}

	void TouchEvent::shutdownPublisher()
	{
	    _publisher->shutdown();
	}
	void TouchEvent::resetPublisher(ros::NodeHandle& nodeHandle)
	{
	    
	    _publisher->reset(nodeHandle);
	}
	
	


    }
}