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


#ifndef LASER_CONVERTER_HPP
#define LASER_CONVERTER_HPP

#include <boost/foreach.hpp>

#include "robot_toolkit/converter/converter_base.hpp"
#include "robot_toolkit/message_actions.h"


#include <sensor_msgs/LaserScan.h>

namespace Sinfonia
{
    namespace Converter
    {
	class LaserConverter : public BaseConverter<LaserConverter>
	{

	    typedef boost::function<void(sensor_msgs::LaserScanPtr)> CallbackT;

	    public:
		LaserConverter( const std::string& name, const float& frequency, const qi::SessionPtr& session );

		void registerCallback( MessageAction::MessageAction action, CallbackT callback );

		void callAll( const std::vector<MessageAction::MessageAction>& actions );

		void reset( );
		
		void setConfig(std::vector<float> configs){}
		
		std::vector<float> setParameters(std::vector<float> parameters)
		{
		    std::vector<float> result;
		    return result;
		}
		std::vector<float> setAllParametersToDefault()
		{
		    std::vector<float> result;
		    return result;
		}
		std::vector<float> getParameters()
		{
		    std::vector<float> result;
		    return result;
		}
		
		void shutdown(){}
		

	    private:
		
		std::vector<float> fromAnyValueToFloatVector(qi::AnyValue& value, std::vector<float>& result);
		qi::AnyObject _pMemory;
		std::map<MessageAction::MessageAction, CallbackT> _callbacks;
		boost::shared_ptr<sensor_msgs::LaserScan> _laserMessage;
		void callLaser();
		
		
	};
    } 
} 

#endif
