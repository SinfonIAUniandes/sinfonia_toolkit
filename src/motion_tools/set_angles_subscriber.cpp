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

#include "robot_toolkit/motion_tools/set_angles_subscriber.hpp"

namespace Sinfonia 
{
    namespace Subscriber 
    {
	
	SetAnglesSubscriber::SetAnglesSubscriber(const std::string& name, const std::string& topic, const qi::SessionPtr& session): 
	  BaseSubscriber(name, topic, session)
	{
	    _topic = topic;
	    _pMotion = session->service("ALMotion");
	    _isInitialized = false;
	    _jointNames.push_back("HeadYaw");
	    _jointNames.push_back("HeadPitch");
	    _jointNames.push_back("LShoulderPitch");
	    _jointNames.push_back("LShoulderRoll");
	    _jointNames.push_back("LElbowYaw");
	    _jointNames.push_back("LElbowRoll");
	    _jointNames.push_back("LWristYaw");
	    _jointNames.push_back("LHand");
	    _jointNames.push_back("HipRoll");
	    _jointNames.push_back("HipPitch");
	    _jointNames.push_back("KneePitch");
	    _jointNames.push_back("RShoulderPitch");
	    _jointNames.push_back("RShoulderRoll");
	    _jointNames.push_back("RElbowYaw");
	    _jointNames.push_back("RElbowRoll");
	    _jointNames.push_back("RWristYaw");
	    _jointNames.push_back("RHand");
	    
	}
	
	void SetAnglesSubscriber::reset(ros::NodeHandle& nodeHandle)
	{
	    _subscriberSetAngles = nodeHandle.subscribe(_topic, 10, &SetAnglesSubscriber::setAnglesCallback, this);
	    _isInitialized = true;
	}
	
	void SetAnglesSubscriber::setAnglesCallback(const robot_toolkit_msgs::set_angles_msg message)
	{
	    if( (message.fraction_max_speed.size() == message.angles.size()) &&  (message.fraction_max_speed.size() == message.names.size()) ) 
	    {
		for(int i=0; i<message.angles.size(); i++)
		{
		    if(checkJointName(message.names[i]))
		    {
			_pMotion.async<void>("setAngles", message.names[i], message.angles[i] , message.fraction_max_speed[i]);
		    }
		    else
		    {
			std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] Error: " <<  message.names[i] << "is not a correct angle name" << RESETCOLOR  << std::endl;
		    }
		}
		    
	    }
	    else
	    {
		std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] Error: Names, Angles and Speed should be the same length " << RESETCOLOR  << std::endl;
	    }
	}
	
	void SetAnglesSubscriber::shutdown()
	{
	    _subscriberSetAngles.shutdown();
	    _isInitialized = false;
	}
	
	std::vector< float > SetAnglesSubscriber::getParameters()
	{

	}
	
	std::vector< float > SetAnglesSubscriber::setParameters(std::vector< float > parameters)
	{

	}
	
	std::vector< float > SetAnglesSubscriber::setDefaultParameters()
	{

	}
	
	bool SetAnglesSubscriber::checkJointName(std::string name)
	{
	    bool result = false;
	    if(std::find(_jointNames.begin(), _jointNames.end(), name) != _jointNames.end())
	    {
		result = true;
	    }
	    return result;
	}


    }
}