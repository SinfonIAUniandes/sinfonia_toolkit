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



#include "robot_toolkit/audio_tools/mic/mic_event.hpp"

namespace Sinfonia
{
    Sinfonia::MicEventRegister::MicEventRegister(const std::string& name, const float& frecuency, const qi::SessionPtr& session)
    {
	_serviceId = 0;
	_speechRecognitionServiceId = 0;
	_speechKey = "WordRecognized";
	_soundKey = "SoundDetected";
	_pAudio = session->service("ALAudioDevice");
	_pRobotModel = session->service("ALRobotModel");
	_pSpeechRecognition = session->service("ALSpeechRecognition");
	_pMemory = session->service("ALMemory");
	_pSoundDetection = session->service("ALSoundDetection");
	_speechRecognitionServiceId = 0;
	_soundDetectionServiceId = 0;
	_session = session;
	_isStarted = false;
	_isPublishing = false;
	_micSampleRate = 48000;
	_channels = 0;
	int micConfig = _pRobotModel.call<int>("_getMicrophoneConfig");
	if(micConfig)
	{
	    _channelMap.push_back(3);
	    _channelMap.push_back(5);
	    _channelMap.push_back(0);
	    _channelMap.push_back(2);
	}
	else
	{
	    _channelMap.push_back(0);
	    _channelMap.push_back(2);
	    _channelMap.push_back(1);
	    _channelMap.push_back(4);
	}
	_publisher = boost::make_shared<Publisher::MicPublisher>();
	_converter = boost::make_shared<Converter::MicConverter>(name, frecuency, session);
	_converter->registerCallback(MessageAction::PUBLISH, boost::bind(&Publisher::MicPublisher::publish, _publisher, _1) );
	_wordList.push_back("robot");
	_confidence = 0.35;
	_counter=0;
    }
    Sinfonia::MicEventRegister::~MicEventRegister()
    {
	stopProcess();
    }

    void MicEventRegister::shutdownEvents()
    {
	//std::cout << "Entra al SHUTDOWN EVENTS" << std::endl;
	//stopSpeechRecognition();
    }

    void Sinfonia::MicEventRegister::resetPublisher(ros::NodeHandle& nodeHandle)
    {
	//_timer = nodeHandle.createTimer( ros::Duration(1.0f), &MicEventRegister::timerCallback, this);
	_counter = 0;
	_publisher->reset(nodeHandle);
    }

    void Sinfonia::MicEventRegister::shutdownPublisher()
    {
	_publisher->shutdown();
    }

    void MicEventRegister::timerCallback(const ros::TimerEvent& event)
    {
	//_timer.stop();
	std::cout << "counter -> " << _counter << std::endl;
	_counter=0;
	//_timer.start();
    }

    void MicEventRegister::initSpeechRecognition()
    {
	boost::mutex::scoped_lock startLock(_mutex);
	std::cout << "Setting up speech Recognition" << std::endl;
	if(!_speechRecognitionServiceId)
	{
	    _pSpeechRecognition.call<void>("setLanguage", "English");
	    _pSpeechRecognition.call<void>("setVocabulary", _wordList, true);
	    _speechServiceName = std::string("ROS-Driver") + _speechKey;
	    _speechRecognitionServiceId = _session->registerService(_speechServiceName, this->shared_from_this());
	    _pSpeechRecognition.call<void>("subscribe", "ROSDriverAudio"+ _speechKey);
	    _pMemory.call<void>("subscribeToEvent", _speechKey.c_str(), _speechServiceName, "wordRecognizedCallback");

	    //_soundServiceName = std::string("ROS-Driver") + _soundKey;
	    //_soundDetectionServiceId = _session->registerService(_soundServiceName, this->shared_from_this());
	    //_pSoundDetection.call<void>("setParameter", "Sensitivity", 0.3f);
	    //_pSoundDetection.call<void>("subscribe", "ROSDriverAudio"+ _soundKey);
	    //_pMemory.call<void>("subscribeToEvent", _soundKey.c_str(), _soundServiceName, "soundDetectionCallback");


	    std::cout << "Speech recognition Initialized" << std::endl;
	}
    }

    void MicEventRegister::stopSpeechRecognition()
    {
	boost::mutex::scoped_lock stopLock(_mutex);
	if(_speechRecognitionServiceId)
	{
	    _session->unregisterService(_speechRecognitionServiceId);
	    _speechRecognitionServiceId = 0;
	}
	_pMemory.call<void>("unsubscribeToEvent", _speechKey.c_str(), _speechServiceName);
	_pSpeechRecognition.call<void>("unsubscribe", "ROSDriverAudio"+ _speechKey);

	if(_soundDetectionServiceId)
	{
	    _session->unregisterService(_soundDetectionServiceId);
	    _soundDetectionServiceId = 0;
	}
	_pMemory.call<void>("unsubscribeToEvent", _soundKey.c_str(), _soundServiceName);
	_pSpeechRecognition.call<void>("unsubscribe", "ROSDriverAudio"+ _soundKey);

    }

    void Sinfonia::MicEventRegister::startProcess()
    {
	boost::mutex::scoped_lock start_lock(_subscriptionMutex);
//	_timer.start();
	if (!_isStarted)
	{
	    if(!_serviceId)
	    {
		_serviceId = _session->registerService("ROS-Driver-Audio", shared_from_this());
		_pAudio.call<void>("setClientPreferences", "ROS-Driver-Audio", _micSampleRate, _channels, 0);
		_pAudio.call<void>("enableEnergyComputation");
		_pAudio.call<void>("subscribe","ROS-Driver-Audio");
		std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] " << "Mic Extractor started" << std::endl;
	    }
	    _isStarted = true;
	    isPublishing(true);
	}
    }
    void Sinfonia::MicEventRegister::stopProcess()
    {
	boost::mutex::scoped_lock stop_lock(_subscriptionMutex);
	if (_isStarted)
	{
	    if(_serviceId)
	    {
		_pAudio.call<void>("unsubscribe", "ROS-Driver-Audio");
		_session->unregisterService(_serviceId);
		_serviceId = 0;
	    }
	    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << "Mic Extractor stopped" << std::endl;
	    _isStarted = false;
	    isPublishing(false);
	}
    }
    void Sinfonia::MicEventRegister::isPublishing(bool state)
    {
	boost::mutex::scoped_lock pub_lock(_processingMutex);
	_isPublishing = state;
    }

    bool Sinfonia::MicEventRegister::isStarted()
    {
	return _isStarted;
    }

    void Sinfonia::MicEventRegister::registerCallback()
    {

    }
    void Sinfonia::MicEventRegister::unregisterCallback()
    {

    }

    void MicEventRegister::setDefaultParameters()
    {
	_micSampleRate = 48000;
	_channels = 0;
    }

    void MicEventRegister::setParameters(std::vector<int> parameters)
    {
	_micSampleRate = parameters[0];
	_channels = parameters[1];
    }

    void Sinfonia::MicEventRegister::processRemote(int numberOfChannels, int samplesByChannel, qi::AnyValue timestamp, qi::AnyValue buffer)
    {
	//naoqi_bridge_msgs::AudioBufferPtr message = naoqi_bridge_msgs::AudioBufferPtr();

	static const boost::shared_ptr<naoqi_bridge_msgs::AudioBuffer> message = boost::make_shared<naoqi_bridge_msgs::AudioBuffer>();

	message->header.stamp = ros::Time::now();
	message->frequency = 48000;
	message->channelMap = _channelMap;

	std::pair<char*, size_t> bufferPointer = buffer.asRaw();

	int16_t* remoteBuffer = (int16_t*)bufferPointer.first;
	int bufferSize = numberOfChannels * samplesByChannel;
	message->data = std::vector<int16_t>(remoteBuffer, remoteBuffer+bufferSize);
	_counter++;
	if(_counter*(4096/_micSampleRate)*1000 > 170.0)
	{
	    _counter = 0;
	    float energy = _pAudio.call<float>("getFrontMicEnergy");
	}
	/*float currentTime  = ros::Time::now().toSec();

	std::cout << BOLDGREEN << "[" <<  currentTime << "] samplesByChannel-> " << samplesByChannel << "frecuency -> " << 1.0f/(currentTime - _lastTime)<< std::endl;
	_lastTime = currentTime;*/

	std::vector<MessageAction::MessageAction> actions;
	boost::mutex::scoped_lock callback_lock(_processingMutex);
	if (_isStarted)
	{
	    if ( _isPublishing && _publisher->isSubscribed() )
	    {
		actions.push_back(MessageAction::PUBLISH);
	    }
	    if (actions.size() >0)
	    {
		_converter->callAll( actions, message );
	    }
	}
    }

    void MicEventRegister::wordRecognizedCallback(std::string key, qi::AnyValue value, std::string subscriberIdentifier)
    {
    	for(unsigned int i = 0; i < value.size() /2 ; ++i)
    	{
    	    std::cout << "word recognized: " << value[i*2].toString() << " with confidence: " << value[i*2+1].toFloat() << std::endl;
    	    if(value[i*2].toString() == "<...> robot <...>" && value[i*2+1].toFloat() > _confidence)
    	    {
    		      std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] " << "its threshold is higher than " << _confidence << std::endl;
    	    }
    	     std::cout << BOLDYELLOW << "[" << ros::Time::now().toSec() << "] speech recognition callback" << std::endl;
    	}
    }

    void MicEventRegister::soundDetectionCallback(std::string key, qi::AnyValue value, std::string subscriberIdentifier)
    {
	std::vector<qi::AnyValue> soundArray =  value.toList<qi::AnyValue>();
	std::cout << BOLDYELLOW << "[" << ros::Time::now().toSec() << "] SoundDetection callcaback value.size() ->" << soundArray.size() << std::endl;
    }
}
