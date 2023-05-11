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


#include "robot_toolkit/audio_tools/speech_recognition/speech_recognition_event.hpp"

namespace Sinfonia
{
    SpeechRecognitionEvent::SpeechRecognitionEvent(const std::string& name, const float& frecuency, const qi::SessionPtr& session)
    {
      _speechKey = "WordRecognized";
      _serviceId = 0;
      _pSpeechRecognition = session->service("ALSpeechRecognition");
    	_pMemory = session->service("ALMemory");
      _session = session;
    }
    SpeechRecognitionEvent::~SpeechRecognitionEvent()
    {

    }

    std::string SpeechRecognitionEvent::startProcess(std::vector<std::string> wordList, float threshold)
    {
    	std::cout << "Setting up speech Recognition" << std::endl;
    	if(!_serviceId)
    	{
          _pSpeechRecognition.call<void>("pause", true);
    	    _pSpeechRecognition.call<void>("setLanguage", "English");
    	    _pSpeechRecognition.call<void>("setVocabulary", wordList, true);
    	    _speechServiceName = std::string("ROS-Driver") + _speechKey;
    	    _serviceId = _session->registerService(_speechServiceName, this->shared_from_this());
    	    _pSpeechRecognition.call<void>("subscribe", "ROSDriverAudio"+ _speechKey);
    	    _pMemory.call<void>("subscribeToEvent", _speechKey.c_str(), _speechServiceName, "wordRecognizedCallback");
    	    std::cout << "Speech recognition Initialized waiting the word" << std::endl;
          _processing = true;
          _threshold = threshold;
          while(_processing)
          {
            std::cout << "waiting the word" << std::endl;
            ros::Duration(0, 500000000).sleep();
          }
    	    _session->unregisterService(_serviceId);
    	    _serviceId = 0;
        	_pMemory.call<void>("unsubscribeToEvent", _speechKey.c_str(), _speechServiceName);
          _pSpeechRecognition.call<void>("pause", false);
        	_pSpeechRecognition.call<void>("unsubscribe", "ROSDriverAudio"+ _speechKey);
      }
      return _wordRecognized;
    }
    void SpeechRecognitionEvent::stopProcess()
    {

    }

    void SpeechRecognitionEvent::wordRecognizedCallback(std::string key, qi::AnyValue value, std::string subscriberIdentifier)
    {
      std::string wordRecognized;
      float highestConfidence = -1;
      for(unsigned int i = 0; i < value.size() /2 ; ++i)
      {
          std::string word = value[i*2].toString();
          word = word.substr(6, word.size() - 12);
          std::cout << word << " confidence -> " << value[i*2+1].toFloat() << std::endl;
          if(value[i*2+1].toFloat() > highestConfidence)
          {
            highestConfidence = value[i*2+1].toFloat();
            wordRecognized = word;
          }
      }
      if(highestConfidence > _threshold)
      {
          _wordRecognized = wordRecognized;
          _processing = false;
          std::cout << "word recognized: " << wordRecognized << std::endl;
      }
      else
      {
        _wordRecognized = "NONE";
        _processing = false;
      }
    }

  }
