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


#ifndef TOOLKIT_HELPERS_HPP
#define TOOLKIT_HELPERS_HPP

#include "robot_toolkit/tools/tools.hpp"

#include <naoqi_bridge_msgs/RobotInfo.h>

#include <naoqi_bridge_msgs/SetString.h>

#include <qi/applicationsession.hpp>

namespace Sinfonia
{
    namespace Helpers
    {
	namespace Toolkit
	{	    

	    const naoqi_bridge_msgs::RobotInfo& getRobotInfo(const qi::SessionPtr& session);

	    bool& setLanguage(const qi::SessionPtr& session, naoqi_bridge_msgs::SetStringRequest request);

	    std::string& getLanguage(const qi::SessionPtr& session);

	}
    }
}

#endif
