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

#ifndef SPEECH_RECOGNITION_EVENT_HPP
#define SPEECH_RECOGNITION_EVENT_HPP

#include <ros/ros.h>
#include <qi/session.hpp>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/enable_shared_from_this.hpp>

namespace Sinfonia
{
    class SpeechRecognitionEvent: public boost::enable_shared_from_this<SpeechRecognitionEvent>
    {
    	public:
    	   SpeechRecognitionEvent(const std::string& name, const float& frecuency, const qi::SessionPtr& session);
    	   ~SpeechRecognitionEvent();

    	   std::string startProcess(std::vector<std::string> wordList, float threshold);
    	   void stopProcess();

    	   void wordRecognizedCallback(std::string key, qi::AnyValue value, std::string subscriberIdentifier);

    	private:
    	    std::string _speechKey;
          std::string _wordRecognized;

          bool _processing;

    	    qi::SessionPtr _session;

    	    boost::mutex _mutex;

    	    qi::AnyObject _pSpeechRecognition;;
    	    qi::AnyObject _pMemory;

    	    std::string _speechServiceName;
          float _threshold;

    	    unsigned int _serviceId;
    };
    QI_REGISTER_OBJECT(SpeechRecognitionEvent, wordRecognizedCallback)
}


#endif
