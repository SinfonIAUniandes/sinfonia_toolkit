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

#include "robot_toolkit/misc_tools/special_settings/special_settings_subscriber.hpp"

namespace Sinfonia
{
    namespace Subscriber
    {
	SpecialSettingsSubscriber::SpecialSettingsSubscriber(const std::string& name, const std::string& topic, const qi::SessionPtr& session):
	  BaseSubscriber(name, topic, session)
	{
	    _topic = topic;
	    _pMotion = session->service("ALMotion");
	    _pBasicAwareness = session->service("ALBasicAwareness");	
	    _isInitialized = false;
	}

	void SpecialSettingsSubscriber::reset(ros::NodeHandle& nodeHandle)
	{
	    _subscriberSpecialSettings = nodeHandle.subscribe(_topic, 10, &SpecialSettingsSubscriber::specialSettingsCallback, this);
	    _isInitialized = true;
	}

	void SpecialSettingsSubscriber::specialSettingsCallback(const robot_toolkit_msgs::special_settings_msg message)
	{
	    if(message.command == "rest")
	    {
		if(message.state)
		{
		    _pMotion.call<void>("rest");
		    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] " << "Going to Rest position" << std::endl;
		}
		else
		{
		    _pMotion.call<void>("wakeUp");
		    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] " << "Going to wakeUp position" << std::endl;
		}
	    }

	    else if(message.command == "external_collision_protection_enabled")
	    {
		_pMotion.call<void>("setExternalCollisionProtectionEnabled", "Move", message.state);
		std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] " << "Setting ExternalCollisionProtectionEnabled to -> " << (int)message.state << std::endl;
	    }

	    else if(message.command == "set_security_distance")
	    {
		_pMotion.call<void>("setOrthogonalSecurityDistance", message.data);
		std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] " << "Setting OrthogonalSecurityDistance to -> " << (float)message.data << std::endl;
	    }

	    else if(message.command == "awareness")
	    {
		_pBasicAwareness.call<void>("setEnabled", message.state);
		std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] " << "Setting awareness to -> " << (int)message.state << std::endl;
	    }
	    else
	    {
		std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << "Special Settings error unknown command" << std::endl;
	    }
	}

	void SpecialSettingsSubscriber::shutdown()
	{
	    _subscriberSpecialSettings.shutdown();
	    _isInitialized = false;
	}

	std::vector< float > SpecialSettingsSubscriber::getParameters()
	{

	}

	std::vector< float > SpecialSettingsSubscriber::setParameters(std::vector< float > parameters)
	{

	}

	std::vector< float > SpecialSettingsSubscriber::setDefaultParameters()
	{

	}
    }
}
