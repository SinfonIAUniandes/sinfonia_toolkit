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

#include "robot_toolkit/navigation_tools/robot_pose/robot_pose_converter.hpp"

#include <boost/foreach.hpp>
#define for_each BOOST_FOREACH

namespace Sinfonia
{
    namespace Converter
    {
	RobotPoseConverter::RobotPoseConverter(const std::string& name, const float& frequency, const qi::SessionPtr& session):
	  BaseConverter( name, frequency, session )
	{
	    _pMemory = session->service("ALMemory");
	}
	void RobotPoseConverter::registerCallback(MessageAction::MessageAction action, RobotPoseConverter::callbackT callback)
	{
	    _callbacks[action] = callback;
	}
	void RobotPoseConverter::callAll(const std::vector< MessageAction::MessageAction >& actions)
	{
	    callRobotPose();
	    for_each(MessageAction::MessageAction action, actions)
	    {
		_callbacks[action](_msgRobotPose);
	    }
	}
	void RobotPoseConverter::reset()
	{
	    
	}
	void RobotPoseConverter::callRobotPose()
	{
	    try 
	    {
		_msgRobotPose = boost::make_shared<geometry_msgs::Vector3>();
		qi::AnyValue value = _pMemory.call<qi::AnyValue>("getData", "NAOqiLocalizer/RobotPose");
		std::vector<float> robotPoseFloatVector = value.toList<float>();
		_msgRobotPose->x = robotPoseFloatVector[0];
		_msgRobotPose->y = robotPoseFloatVector[1];
		_msgRobotPose->z = robotPoseFloatVector[2];
	    } 
	    catch (qi::FutureUserException)
	    {
		_msgRobotPose = boost::make_shared<geometry_msgs::Vector3>();
		_msgRobotPose->x = NAN;
		_msgRobotPose->y = NAN;
		_msgRobotPose->z = NAN;
		std::cout << BOLDYELLOW << "[" << ros::Time::now().toSec() << "] NAOqiLocalizer/RobotPose not available" << std::endl;
	    }
	}
    }
}
