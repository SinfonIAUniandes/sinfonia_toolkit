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




#ifndef AUDIO_EVENT_HPP
#define AUDIO_EVENT_HPP

#include <ros/ros.h>
#include <qi/session.hpp>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/enable_shared_from_this.hpp>

#include "mic_publisher.hpp"
#include "mic_converter.hpp"

namespace Sinfonia
{
    class MicEventRegister: public boost::enable_shared_from_this<MicEventRegister>
    {
	public:
	   MicEventRegister(const std::string& name, const float& frecuency, const qi::SessionPtr& session);
	   ~MicEventRegister();

	   void resetPublisher(ros::NodeHandle& nodeHandle);
	   void shutdownPublisher();

	   void startProcess();
	   void stopProcess();
	   void isPublishing(bool state);

	   bool isStarted();

	   void setDefaultParameters();
	   void setParameters(std::vector<int> parameters);
	   void shutdownEvents();
	   void processRemote(int numberOfChannels, int samplesByChannel, qi::AnyValue timestamp, qi::AnyValue buffer);
	   void wordRecognizedCallback(std::string key, qi::AnyValue value, std::string subscriberIdentifier);
	   void soundDetectionCallback(std::string key, qi::AnyValue value, std::string subscriberIdentifier);

	private:

	    void timerCallback(const ros::TimerEvent& event);

	    void registerCallback();
	    void unregisterCallback();
	    void initSpeechRecognition();
	    void stopSpeechRecognition();

	    boost::shared_ptr<Converter::MicConverter> _converter;
	    boost::shared_ptr<Publisher::MicPublisher> _publisher;

	    std::string _speechKey;
	    std::string _soundKey;


	    qi::SessionPtr _session;

	    boost::mutex _mutex;

	    qi::AnyObject _pAudio;
	    qi::AnyObject _pRobotModel;
	    qi::AnyObject _pSpeechRecognition;
	    qi::AnyObject _pSoundDetection;
	    qi::AnyObject _pMemory;

	    qi::FutureSync<qi::AnyObject> _pAudioExtractorRequest;

	    std::vector<uint8_t> _channelMap;
	    std::vector<std::string> _wordList;

	    std::string _speechServiceName;
	    std::string _soundServiceName;

	    unsigned int _serviceId;
	    unsigned int _speechRecognitionServiceId;
	    unsigned int _soundDetectionServiceId;

	    boost::mutex _subscriptionMutex;
	    boost::mutex _processingMutex;


	    ros::Timer _timer;

	    int _counter;

	    bool _isStarted;
	    bool _isPublishing;

	    int _micSampleRate;
	    int _channels;

	    float _confidence;
    };
    QI_REGISTER_OBJECT(MicEventRegister, processRemote, soundDetectionCallback)
}


#endif
