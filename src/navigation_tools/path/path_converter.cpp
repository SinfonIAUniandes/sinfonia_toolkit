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

#include "robot_toolkit/navigation_tools/path/path_converter.hpp"
#include <boost/foreach.hpp>
#define for_each BOOST_FOREACH

namespace Sinfonia
{
    namespace Converter
    {
	PathConverter::PathConverter(const std::string& name, const float& frequency, const qi::SessionPtr& session):
	  BaseConverter( name, frequency, session )
	{
	    _pMemory = session->service("ALMemory");
	}
	
	void PathConverter::registerCallback(MessageAction::MessageAction action, PathConverter::callbackT callback)
	{
	    _callbacks[action] = callback;
	}
	
	void PathConverter::callAll(const std::vector< MessageAction::MessageAction >& actions)
	{
	    callPath();
	    for_each(MessageAction::MessageAction action, actions)
	    {
		_callbacks[action](_msgPath);
	    }
	}
	
	void PathConverter::reset()
	{
	    
	}
	
	void PathConverter::callPath()
	{
	    try
	    {
		_msgPath = boost::make_shared<robot_toolkit_msgs::path_msg>();
		_msgPath->point_world.clear();
		qi::AnyValue value = _pMemory.call<qi::AnyValue>("getData", "NAOqiPlanner/Path");
		std::vector<float> pathVector = value.toList<float>();
		for (size_t i=0; i<(pathVector.size()/3); i++)
		{
		    Eigen::Vector3f pointWorld = Eigen::Vector3f(pathVector[3*i], pathVector[3*i+1], pathVector[3*i+2]);
		    geometry_msgs::Vector3 point;
		    point.x = pointWorld[0];
		    point.y = pointWorld[1];
		    point.z = pointWorld[2];
		    _msgPath->point_world.push_back(point);    
		}
		
	    } 
	    catch (qi::FutureUserException) 
	    {
		_msgPath->point_world.clear();
		std::cout << BOLDYELLOW << "[" << ros::Time::now().toSec() << "] NAOqiPlanner/Path not available" << std::endl;
	    }
	}


    }   
}