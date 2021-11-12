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

#include "robot_toolkit/audio_tools/mic/mic_converter.hpp"

#define for_each BOOST_FOREACH

namespace Sinfonia
{
    namespace Converter
    {
	MicConverter::MicConverter(const std::string& name, const float& frequency, const qi::SessionPtr& session):
	  BaseConverter( name, frequency, session )
	{
	    
	}
	
	MicConverter::~MicConverter()
	{

	}

	
	void MicConverter::registerCallback(MessageAction::MessageAction action, MicConverter::CallbackT callback)
	{
	    _callbacks[action] = callback;
	}
	
	void MicConverter::callAll(const std::vector< MessageAction::MessageAction >& actions, naoqi_bridge_msgs::AudioBufferPtr message)
	{
	    _message = message;
	    for_each(MessageAction::MessageAction action, actions )
	    {
		_callbacks[action](_message);
	    }
	}
	
	void MicConverter::reset()
	{

	}

	
    }
}
