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

#include "robot_toolkit/audio_tools/mic/mic_localization_event.hpp"

namespace Sinfonia 
{
    MicLocalizationEvent::MicLocalizationEvent(std::string name, const float& frecuency, const qi::SessionPtr& session)
    {
	_session = session;
	_pMemory = session->service("ALMemory");
	_key = "ALSoundLocalization/SoundLocated";
	_name = name;
	_serviceId = 0;
	_isStarted = false;
	_publisher = boost::make_shared<Publisher::MicLocalizationPublisher>("/audio_localization");
    }
    
    void MicLocalizationEvent::startProcess()
    {
	boost::mutex::scoped_lock startLock(_mutex);
	if( !_isStarted )
	{
	    if( !_serviceId )
	    {
		std::string serviceName = std::string("ROS-Driver") + _key;
		_serviceId = _session->registerService(serviceName, this->shared_from_this());
		_pMemory.call<void>("subscribeToEvent",_key.c_str(), serviceName, "soundLocatedCallback");		
	    }
	    _isStarted = true;
	}
    }
    
    void MicLocalizationEvent::stopProcess()
    {
	boost::mutex::scoped_lock stopLock(_mutex);
	if(_isStarted)
	{
	    std::string serviceName = std::string("ROS-Driver") + _key;
	    if(_serviceId)
	    {
		_session->unregisterService(_serviceId);
		_serviceId = 0;
	    }
	    _isStarted = false;
	}
    }
    
    void MicLocalizationEvent::soundLocatedCallback(std::string key, qi::AnyValue value, std::string subscriberIdentifier)
    {
	static const boost::shared_ptr<robot_toolkit_msgs::audio_localization_msg> message = boost::make_shared<robot_toolkit_msgs::audio_localization_msg>();
	std::vector<float> data = value[1].toList<float>();
	std::vector<float> headFrameTorso = value[2].toList<float>();
	std::vector<float> headFrameRobot = value[3].toList<float>();
	message->header.stamp = ros::Time::now();
	message->azimuth = data[0];
	message->elevation = data[1];
	message->confidendce = data[2];
	message->energy = data[3];
	for( int i=0; i<headFrameTorso.size(); i++ )
	{
	    message->head_positions_in_frame_torso.push_back(headFrameTorso[i]);
	}
	for( int i=0; i<headFrameRobot.size(); i++ )
	{
	    message->head_positions_in_frame_robot.push_back(headFrameRobot[i]);
	}
	_publisher->publish(message);
	message->head_positions_in_frame_torso.clear();
	message->head_positions_in_frame_robot.clear();
    }
    
    void MicLocalizationEvent::resetPublisher(ros::NodeHandle& nodeHandle)
    {
	_publisher->reset(nodeHandle);
    }

    void MicLocalizationEvent::shutdownPublisher()
    {
	_publisher->shutdown();
    }

}
