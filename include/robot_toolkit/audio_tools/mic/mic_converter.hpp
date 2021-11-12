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


#ifndef AUDIO_CONVERTER_HPP
#define AUDIO_CONVERTER_HPP

#include <boost/foreach.hpp>

#include "robot_toolkit/converter/converter_base.hpp"
#include "robot_toolkit/message_actions.h"


#include <naoqi_bridge_msgs/AudioBuffer.h>
#include <qi/anymodule.hpp>

namespace Sinfonia
{
    namespace Converter
    {

	class MicConverter : public BaseConverter<MicConverter>
	{

	    typedef boost::function<void( naoqi_bridge_msgs::AudioBufferPtr )> CallbackT;

	    public:
		
		MicConverter( const std::string& name, const float& frequency, const qi::SessionPtr& session );
		~MicConverter();

		void registerCallback( MessageAction::MessageAction action, CallbackT callback );

		void callAll( const std::vector<MessageAction::MessageAction>& actions, naoqi_bridge_msgs::AudioBufferPtr message );

		void reset();
		
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
		
	    private:
		
		std::map<MessageAction::MessageAction, CallbackT> _callbacks;
		naoqi_bridge_msgs::AudioBufferPtr _message;
	};

    } 
} 

#endif
