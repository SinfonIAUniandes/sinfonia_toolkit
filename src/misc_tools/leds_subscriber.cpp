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

#include "robot_toolkit/misc_tools/leds_subscriber.hpp"

namespace Sinfonia 
{
    namespace Subscriber 
    {
	
	LedsSubscriber::LedsSubscriber(const std::string& name, const std::string& topic, const qi::SessionPtr& session):
	    BaseSubscriber(name, topic, session)
	{
	    _topic = topic;
	    _pLeds = session->service("ALLeds");
	    _isInitialized = false;
	}
	void LedsSubscriber::reset(ros::NodeHandle& nodeHandle)
	{
	    _subscriberLeds = nodeHandle.subscribe(_topic, 10, &LedsSubscriber::ledsCallback , this);
	    _isInitialized = true;
	}
	void LedsSubscriber::ledsCallback(const robot_toolkit_msgs::leds_parameters_msg message)
	{
	    float redColor = ((float)message.red / 255.0);
	    float greenColor = ((float)message.green / 255.0) ;
	    float blueColor = ((float)message.blue / 255.0);
	    
	    if (message.name.find("Ear") != std::string::npos) 
	    {
		try
		{
		    _pLeds.call<void>("fadeRGB", message.name, 0, 0, blueColor, message.time);  
		}
		catch( std::exception& e)
		{
		    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << "Could not parse" << message.name << " as a LED Device, RGB-LED or Group" << std::endl;
		}
	    }
	    else
	    {
		try
		{
		    _pLeds.call<void>("fadeRGB", message.name, redColor, greenColor, blueColor, message.time);
		}
		catch( std::exception& e)
		{
		    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << "Could not parse" << message.name << " as a LED Device, RGB-LED or Group" << std::endl;
		}
	    }
	}
	void LedsSubscriber::shutdown()
	{
	     _subscriberLeds.shutdown();
	    _isInitialized = false;
	}
	std::vector< float > LedsSubscriber::getParameters()
	{
	    std::vector<float> result;
	    return result;
	}
	std::vector< float > LedsSubscriber::setParameters(std::vector< float > parameters)
	{
	    return getParameters();
	}
	std::vector< float > LedsSubscriber::setDefaultParameters()
	{
	    return getParameters();
	}
    }
}