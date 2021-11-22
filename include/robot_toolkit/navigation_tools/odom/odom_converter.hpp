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


#ifndef ODOM_CONVERTER_HPP
#define ODOM_CONVERTER_HPP

#include "robot_toolkit/converter/converter_base.hpp"
#include "robot_toolkit/tools/robot_description.hpp"


#include "robot_toolkit/message_actions.h"

#include <urdf/model.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/buffer.h>
#include <robot_state_publisher/robot_state_publisher.h>

namespace Sinfonia
{
    namespace Converter
    {

	class OdomConverter : public BaseConverter<OdomConverter>
	{
	    typedef boost::function<void(nav_msgs::OdometryPtr)> callbackT;

	    public:
		OdomConverter( const std::string& name, const float& frequency, const qi::SessionPtr& session );
		void registerCallback( MessageAction::MessageAction action, callbackT callback );
		void callAll( const std::vector<MessageAction::MessageAction>& actions );
		void reset( );
		
		void setConfig(std::vector<float> configs){}
		
		std::vector<float> setParameters(std::vector<float> parameters){}
		std::vector<float> setAllParametersToDefault(){}
		std::vector<float> getParameters(){}
		
		void shutdown(){}

	    private:
		qi::AnyObject _pMotion;
		std::map<MessageAction::MessageAction, callbackT> _callbacks;
		boost::shared_ptr<nav_msgs::Odometry> _msgOdom;
		
		void callOdom();
	};

    }
}

#endif
