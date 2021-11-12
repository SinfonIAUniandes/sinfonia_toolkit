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


#include "robot_toolkit/navigation_tools/cmd_vel/cmd_vel_subscriber.hpp"

namespace Sinfonia 
{
    namespace Subscriber 
    {

	CmdVelSubscriber::CmdVelSubscriber(const std::string& name, const std::string& cmdVelTopic, const qi::SessionPtr& session):
	BaseSubscriber(name, cmdVelTopic, session)
	{
	    _cmdVelTopic = cmdVelTopic;
	    _pMotion = _session-> service("ALMotion");
	    _securityTime = 0.5;
	}
	
	void CmdVelSubscriber::reset(ros::NodeHandle& nodeHandle)
	{ 
	    if (_securityTime != -1)
	    {
		std::cout << BOLDCYAN << "[" << ros::Time::now().toSec() << "] " << "Setting security time to: " << _securityTime << " s" << std::endl;
		_timer = nodeHandle.createTimer( ros::Duration(_securityTime), &CmdVelSubscriber::timerCallback, this);
		_timer.stop();
	    }
	    else
	    {
		std::cout << BOLDCYAN << "[" << ros::Time::now().toSec() << "] " << "Turning off security time" << std::endl;
	    }
	    _subscriberCmdVel = nodeHandle.subscribe( _cmdVelTopic, 10, &CmdVelSubscriber::cmdVelCallback, this );	
	    _isInitialized = true;
	}

	void CmdVelSubscriber::cmdVelCallback(const geometry_msgs::TwistConstPtr& twistMsg)
	{
	    _timer.stop();
	    const float& velX = twistMsg->linear.x;
	    const float& velY = twistMsg->linear.y;
	    const float& velTh = twistMsg->angular.z;
	    std::cout << BOLDCYAN << "[" << ros::Time::now().toSec() << "] " << "Going to move x: " << velX << " y: " << velY << " th: " << velTh << std::endl;
	    _pMotion.async<void>("move", velX, velY, velTh );
	    if (_securityTime != -1)
	    {
		_timer.start();
	    }
	}
	
	void CmdVelSubscriber::shutdown()
	{
	    _subscriberCmdVel.shutdown();
	    _isInitialized = false;
	    _timer.stop();
	}
	
	std::vector< float > CmdVelSubscriber::getParameters()
	{
	    std::vector<float> result;
	    result.push_back(_securityTime);
	    return result;
	}
	
	std::vector< float > CmdVelSubscriber::setParameters(std::vector<float> parameters)
	{
	    _securityTime = parameters[0];
	    return getParameters();
	}
	
	std::vector< float > CmdVelSubscriber::setDefaultParameters()
	{
	    _securityTime = 0.5;
	    return getParameters();
	}

	void CmdVelSubscriber::timerCallback(const ros::TimerEvent& event)
	{ 
	    std::cout << BOLDYELLOW << "[" << ros::Time::now().toSec() << "] " << "Stopping Robot due to security timeout" << std::endl;
	    _pMotion.async<void>("move", 0, 0, 0 );
	    _timer.stop();
	}
    }
}