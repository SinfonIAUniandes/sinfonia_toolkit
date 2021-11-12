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

#include "robot_toolkit/navigation_tools/laser/naoqi_depth2laser_converter.hpp"

#include <boost/foreach.hpp>
#define for_each BOOST_FOREACH

namespace Sinfonia
{
    namespace Converter
    {
	
	NaoqiDepth2LaserConverter::NaoqiDepth2LaserConverter(const std::string& name, const float& frequency, const qi::SessionPtr& session):
	  BaseConverter( name, frequency, session )
	{
	    _pMemory = session->service("ALMemory");
	}
	void NaoqiDepth2LaserConverter::registerCallback(MessageAction::MessageAction action, NaoqiDepth2LaserConverter::CallbackT callback)
	{
	    _callbacks[action] = callback;
	}
	void NaoqiDepth2LaserConverter::callAll(const std::vector< MessageAction::MessageAction >& actions)
	{
	    callDepth2Laser();
	    for_each(MessageAction::MessageAction action, actions)
	    {
		_callbacks[action](_msg);
	    }
	}
	void NaoqiDepth2LaserConverter::reset()
	{
	    
	}
	void NaoqiDepth2LaserConverter::callDepth2Laser()
	{
	    _msg = boost::make_shared<sensor_msgs::LaserScan>();
	    _msg->header.frame_id = "base_footprint";
	    _msg->header.stamp = ros::Time::now();
	    try
	    {
		qi::AnyValue rangesReadings = _pMemory.call<qi::AnyValue>("getData", "NAOqiDepth2Laser/Ranges");
		std::vector<float> ranges = rangesReadings.toList<float>();
		_msg->ranges.resize(ranges.size());
		for(int i=0; i<ranges.size(); i++)
		    _msg->ranges[i] = ranges.at(i);
		qi::AnyValue minAngleReading = _pMemory.call<qi::AnyValue>("getData", "NAOqiDepth2Laser/MinAngle");
		_msg->angle_min = minAngleReading.toFloat();
		qi::AnyValue maxAngleReading = _pMemory.call<qi::AnyValue>("getData", "NAOqiDepth2Laser/MaxAngle");
		_msg->angle_max = maxAngleReading.toFloat();
		qi::AnyValue numRangesReading = _pMemory.call<qi::AnyValue>("getData", "NAOqiDepth2Laser/NumRanges");
		float numRanges = numRangesReading.toFloat();
		_msg->angle_increment = (maxAngleReading.toFloat() - minAngleReading.toFloat())/numRanges;
		_msg->scan_time = 0.01;
		_msg->time_increment = 0.01/numRanges;
		_msg->range_min = 0.1;
		qi::AnyValue maxRangeReading = _pMemory.call<qi::AnyValue>("getData", "NAOqiDepth2Laser/MaxRange");
		_msg->range_max = maxRangeReading.toFloat();
		
	    }
	    catch (qi::FutureUserException)
	    {
		std::cout << BOLDYELLOW << "[" << ros::Time::now().toSec() << "] NAOqiDepth2Laser not available" << std::endl;
	    }
	}
    }
}
