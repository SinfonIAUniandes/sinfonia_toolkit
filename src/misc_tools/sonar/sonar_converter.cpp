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

#include "robot_toolkit/misc_tools/sonar/sonar_converter.hpp"
#include <boost/foreach.hpp>
#define for_each BOOST_FOREACH

namespace Sinfonia 
{
    namespace Converter 
    {
	SonarConverter::SonarConverter(const std::string& name, const float& frequency, const qi::SessionPtr& session):
	  BaseConverter( name, frequency, session )
	{
	    _pMemory = session->service("ALMemory");
	    _pSonar = session->service("ALSonar");
	    _isSubscribed = false;
	    
	    std::vector<std::string> keys;
	    keys.push_back("Device/SubDeviceList/Platform/Front/Sonar/Sensor/Value");
	    keys.push_back("Device/SubDeviceList/Platform/Back/Sonar/Sensor/Value");
	    _frames.push_back("SonarFront_frame");
	    _frames.push_back("SonarBack_frame");
	    
	    _msgs.resize(_frames.size());
	    for(size_t i = 0; i < _msgs.size(); ++i)
	    {
		_msgs[i].header.frame_id = _frames[i];
		_msgs[i].min_range = 0.25;
		_msgs[i].max_range = 2.55;
		_msgs[i].field_of_view = 0.523598776;
		_msgs[i].radiation_type = sensor_msgs::Range::ULTRASOUND;
	    }
	    _keys.resize(keys.size());
	    size_t i = 0;
	    for(std::vector<std::string>::const_iterator it = keys.begin(); it != keys.end(); ++it, ++i)
		_keys[i] = *it;
	}
	
	SonarConverter::~SonarConverter()
	{
	    if(_isSubscribed)
	    {
		_pSonar.call<void>("unsubscribe", "ROS");
		_isSubscribed = false;
	    }
	}

	void SonarConverter::registerCallback(MessageAction::MessageAction action, SonarConverter::CallbackT callback)
	{
	    _callbacks[action] = callback;
	}
	
	void SonarConverter::callAll(const std::vector< MessageAction::MessageAction >& actions)
	{
	    if (!_isSubscribed)
	    {
		_pSonar.call<void>("subscribe", "ROS");
		_isSubscribed = true;
	    }

	    std::vector<float> values;
	    try 
	    {
		qi::AnyValue anyvalues = _pMemory.call<qi::AnyValue>("getListData", _keys);
		fromAnyValueToFloatVector(anyvalues, values);
	    } 
	    catch (const std::exception& e) 
	    {
		std::cerr << "Exception caught in SonarConverter: " << e.what() << std::endl;
		return;
	    }
	    ros::Time now = ros::Time::now();
	    for(size_t i = 0; i < _msgs.size(); ++i)
	    {
		_msgs[i].header.stamp = now;
		_msgs[i].range = float(values[i]);
	    }

	    for_each( MessageAction::MessageAction action, actions )
	    {
		_callbacks[action]( _msgs );
	    }
	}
	
	std::vector<float> SonarConverter::fromAnyValueToFloatVector(qi::AnyValue& value, std::vector< float >& result)
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

	void SonarConverter::reset()
	{

	}
    }
}