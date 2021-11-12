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

#include "robot_toolkit/audio_tools/speech/speech_subscriber.hpp"

namespace Sinfonia 
{
    namespace Subscriber 
    {
	SpeechSubscriber::SpeechSubscriber(const std::string& name, const std::string& topic, const qi::SessionPtr& session):
	  BaseSubscriber(name, topic, session)
	{
	    _speechTopic = topic;
	    _pTextToSpeech = session->service("ALTextToSpeech");
	    _pTextToSpeechAnimated = session->service("ALAnimatedSpeech");
	    _isInitialized = false;
	    _language = "English";
	    _pTextToSpeech.call<void>("setLanguage", _language);
	    
	}
		
	void SpeechSubscriber::reset(ros::NodeHandle& nodeHandle)
	{
	    _subscriberSpeech = nodeHandle.subscribe(_speechTopic, 10, &SpeechSubscriber::speechCallback, this);
	    _isInitialized = true;
	}
	void SpeechSubscriber::speechCallback(const robot_toolkit_msgs::speech_msg& message)
	{
	    if(message.language == "Spanish" || message.language == "English")
	    {
		if(message.language != _language)
		{
		    _language = message.language;
		    _pTextToSpeech.call<void>("setLanguage", message.language);
		}
		if ( message.animated )
		{
		    _pTextToSpeechAnimated.async<void>("say", message.text);
		    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] Robot saying: " << message.text << std::endl;
		}
		else
		{
		    _pTextToSpeech.async<void>("say", message.text);
		    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] NAOqiPlanner/Path not available" << std::endl;
		    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] Robot saying: " << message.text << std::endl;
		}
	    }
	    else
	    {
		if(_language == "Spanish")
		{
		    _pTextToSpeech.async<void>("say", "Lo siento, no se hablar en ese idioma");
		}
		else
		{
		    _pTextToSpeech.async<void>("say", "I am sorry, I dont know that language");
		}
	    }
	}
	void SpeechSubscriber::shutdown()
	{
	    _subscriberSpeech.shutdown();
	    _isInitialized = false;
	}
	
	std::vector<float> SpeechSubscriber::getParameters()
	{
	    std::vector<float> result;
	    result.push_back(_pTextToSpeech.call<float>("getParameter", "pitchShift"));
	    result.push_back(_pTextToSpeech.call<float>("getParameter", "doubleVoice"));
	    result.push_back(_pTextToSpeech.call<float>("getParameter", "doubleVoiceLevel"));
	    result.push_back(_pTextToSpeech.call<float>("getParameter", "doubleVoiceTimeShift"));
	    result.push_back(_pTextToSpeech.call<float>("getParameter", "speed"));
	    return result;
	}
	std::vector<float> SpeechSubscriber::setDefaultParameters()
	{
	    if(_language == "English")
	    {
		_pTextToSpeech.call<void>("setParameter", "pitchShift", 1.170f);
		_pTextToSpeech.call<void>("setParameter", "doubleVoice", 0.0f);
		_pTextToSpeech.call<void>("setParameter", "doubleVoiceLevel", 0.0f);
		_pTextToSpeech.call<void>("setParameter", "doubleVoiceTimeShift", 0.0f);
		_pTextToSpeech.call<void>("setParameter", "speed", 100.0f);		
	    }
	    else
	    {
		_pTextToSpeech.call<void>("setParameter", "pitchShift", 1.25f);
		_pTextToSpeech.call<void>("setParameter", "doubleVoice", 0.0f);
		_pTextToSpeech.call<void>("setParameter", "doubleVoiceLevel", 0.0f);
		_pTextToSpeech.call<void>("setParameter", "doubleVoiceTimeShift", 0.0f);
		_pTextToSpeech.call<void>("setParameter", "speed", 90.0f);
	    }
	    return getParameters();
	}
	std::vector< float > SpeechSubscriber::setParameters(std::vector< float > parameters)
	{
	    _pTextToSpeech.call<void>("setParameter", "pitchShift", parameters[0]);
	    _pTextToSpeech.call<void>("setParameter", "doubleVoice", parameters[1]);
	    _pTextToSpeech.call<void>("setParameter", "doubleVoiceLevel", parameters[2]);
	    _pTextToSpeech.call<void>("setParameter", "doubleVoiceTimeShift", parameters[3]);
	    _pTextToSpeech.call<void>("setParameter", "speed", parameters[4]);
	    return getParameters();
	}
	
		void SpeechSubscriber::printSpeechParams(robot_toolkit_msgs::speech_parameters_msg parameters)
	{
	    std::cout << BOLDCYAN << "Pitch Shift: " <<  parameters.pitch_shift << std::endl;
	    std::cout << BOLDCYAN << "Double Voice: " <<  parameters.double_voice << std::endl;
	    std::cout << BOLDCYAN << "Double Voice Level: " <<  parameters.double_voice_level << std::endl;
	    std::cout << BOLDCYAN << "Double Voice Time Shift: " <<  parameters.double_voice_time_shift << std::endl;
	    std::cout << BOLDCYAN << "Speed: " <<  parameters.speed << std::endl;
	}
	
	robot_toolkit_msgs::speech_parameters_msg SpeechSubscriber::toSpeechParameters(std::vector< float > params)
	{
	    robot_toolkit_msgs::speech_parameters_msg result;
	
	    result.pitch_shift = params[0];
	    result.double_voice = params[1];
	    result.double_voice_level = params[2];
	    result.double_voice_time_shift = params[3];
	    result.speed = params[4];
	    
	    return result;
	}

    }
}