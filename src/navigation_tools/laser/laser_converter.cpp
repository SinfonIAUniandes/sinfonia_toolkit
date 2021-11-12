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

#include "robot_toolkit/navigation_tools/laser/laser_converter.hpp"

#define for_each BOOST_FOREACH

namespace Sinfonia
{
    namespace Converter
    {
	static const char* _laserMemoryKeys[] = {
					    // RIGHT LASER
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg01/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg01/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg02/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg02/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg03/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg03/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg04/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg04/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg05/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg05/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg06/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg06/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg07/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg07/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg08/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg08/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg09/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg09/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg10/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg10/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg11/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg11/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg12/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg12/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg13/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg13/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg14/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg14/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg15/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg15/Y/Sensor/Value",
					    // FRONT LASER
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg01/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg01/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg02/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg02/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg03/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg03/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg04/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg04/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg05/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg05/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg06/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg06/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg07/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg07/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg08/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg08/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg09/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg09/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg10/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg10/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg11/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg11/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg12/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg12/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg13/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg13/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg14/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg14/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg15/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg15/Y/Sensor/Value",
					    // LEFT LASER
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg01/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg01/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg02/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg02/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg03/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg03/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg04/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg04/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg05/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg05/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg06/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg06/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg07/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg07/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg08/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg08/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg09/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg09/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg10/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg10/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg11/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg11/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg12/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg12/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg13/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg13/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg14/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg14/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg15/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg15/Y/Sensor/Value",
							};
							
	LaserConverter::LaserConverter(const std::string& name, const float& frequency, const qi::SessionPtr& session):
	    BaseConverter( name, frequency, session )
	{
	    _pMemory = _session->service("ALMemory");
	    _laserMessage = boost::make_shared<sensor_msgs::LaserScan>();	    
	}
	
	void LaserConverter::registerCallback(MessageAction::MessageAction action, LaserConverter::CallbackT callback)
	{
	    _callbacks[action] = callback;
	}

	void LaserConverter::callAll(const std::vector<MessageAction::MessageAction>& actions)
	{
	    double init = ros::Time::now().toSec();
	    callLaser();
	    for_each(MessageAction::MessageAction action, actions)
	    {
		_callbacks[action](_laserMessage);		
	    }
	    //std::cout << BOLDYELLOW << "Topic: /laser " << BOLDCYAN << "elapsed time (s): "<< std::fixed << std::setprecision(8) << ros::Time::now().toSec() - init << std::endl;
	}
	
	

	void LaserConverter::callLaser()
	{
	    _laserMessage = boost::make_shared<sensor_msgs::LaserScan>();
	    static const std::vector<std::string> laserKeysValue(_laserMemoryKeys, _laserMemoryKeys+90);
	    std::vector<float> resultValue;
	    try
	    {
		qi::AnyValue anyvalues = _pMemory.call<qi::AnyValue>("getListData", laserKeysValue);
		fromAnyValueToFloatVector(anyvalues, resultValue);
	    }
	    catch (const std::exception& e) 
	    {
		std::cerr << "Exception caught in LaserConverter: " << e.what() << std::endl;
		return;
	    }
	    _laserMessage->header.stamp = ros::Time::now();
	    _laserMessage->header.frame_id = "base_link";
	    _laserMessage->angle_min = -2.0944;
	    _laserMessage->angle_max = 2.0944;   
	    _laserMessage->angle_increment = (2*2.0944) / (15+15+15+8+8); 
	    _laserMessage->range_min = 0.1;
	    _laserMessage->range_max = 1.5;
	    _laserMessage->ranges.resize(61);
	    _laserMessage->ranges = std::vector<float>(61, -1.0f);
	    size_t pos = 0;

	    for( size_t i=0; i<30; i=i+2, ++pos)
	    {
		const float lx = resultValue[28-i]; 
		const float ly = resultValue[28-i+1];
		float bx = lx*std::cos(-1.757) - ly*std::sin(-1.757) - 0.018;
		float by = lx*std::sin(-1.757) + ly*std::cos(-1.757) - 0.090;
		float dist = std::sqrt( std::pow(bx,2) + std::pow(by,2) );
		_laserMessage->ranges[pos] = dist;
	    }

	    pos = pos+8;

	    for( size_t i=0; i<30; i=i+2, ++pos)
	    {
		const float lx = resultValue[58-i];
		const float ly = resultValue[58-i+1];
		float bx = lx + 0.056 ;
		float by = ly;
		float dist = std::sqrt( std::pow(bx,2) + std::pow(by,2) );
		_laserMessage->ranges[pos] = dist;
	    }

	    pos = pos+8;

	    for( size_t i=0; i<30; i=i+2, ++pos)
	    {
		const float lx = resultValue[88-i];
		const float ly = resultValue[88-i+1];
		float bx = lx*std::cos(1.757) - ly*std::sin(1.757) - 0.018;
		float by = lx*std::sin(1.757) + ly*std::cos(1.757) + 0.090;
		float dist = std::sqrt( std::pow(bx,2) + std::pow(by,2) );
		_laserMessage->ranges[pos] = dist;
	    }
	}
	
	
	void LaserConverter::reset()
	{
	    _laserMessage->header.frame_id = "base_footprint";
	    _laserMessage->angle_min = -2.0944;
	    _laserMessage->angle_max = 2.0944;   
	    _laserMessage->angle_increment = (2*2.0944) / (15+15+15+8+8); 
	    _laserMessage->range_min = 0.1;
	    _laserMessage->range_max = 1.5;
	    _laserMessage->ranges = std::vector<float>(61, -1.0f);
	}

	std::vector< float > LaserConverter::fromAnyValueToFloatVector(qi::AnyValue& value, std::vector< float >& result)
	{
	    qi::AnyReferenceVector anyrefs = value.asListValuePtr();
	    for(int i=0; i<anyrefs.size();i++)
	    {
		try
		{
		    result.push_back(anyrefs[i].content().toFloat());
		}
		catch(std::runtime_error& e)
		{
		    result.push_back(-1.0);
		    std::cout << e.what() << "=> set to -1" << std::endl;
		}
	    }
	    return result;
	}
	
    }
}
