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



#include "robot_toolkit/robot_toolkit.hpp"
#include "robot_toolkit/message_actions.h"

#include "robot_toolkit/ros_environment.hpp"



#include <boost/foreach.hpp>
#define for_each BOOST_FOREACH

namespace Sinfonia
{
    RobotToolkit::RobotToolkit(qi::SessionPtr session, const std::string& prefix)
    {
		if(prefix == "")
		{
			std::cout << "Error driver prefix must not be empty" << std::endl;
			throw new ros::Exception("Error driver prefix must not be empty");
		}
		else
		{
			Sinfonia::RosEnvironment::setPrefix(prefix);
		}
		_sessionPtr = session;
		_isRosLoopEnabled = true;
		_publishOdom = false;
	_speechRecognition  = boost::make_shared<Sinfonia::SpeechRecognitionEvent>("speech_recognition", 10.0f, _sessionPtr);
	}

    RobotToolkit::~RobotToolkit()
    {
		std::cout << BOLDYELLOW << "[" << ros::Time::now().toSec() << "] " << "robot_toolkit is shutting down.." << std::endl;
		if(_nodeHandlerPtr)
		{
			_nodeHandlerPtr->shutdown();
			ros::shutdown();
		}
    }

	void RobotToolkit::setPublishOdom(){
		_publishOdom = true;
	}

    std::string RobotToolkit::_whoWillWin()
    {
	return "SinfonIA SSPL Robocup Team";
    }

    void RobotToolkit::init()
    {
	ros::Time::init();
	registerDefaultConverter();
	registerDefaultSubscriber();
    }

    void RobotToolkit::rosLoop()
    {
	static std::vector<MessageAction::MessageAction> actions;
	while(_isRosLoopEnabled)
	{
	    actions.clear();
	    {
		boost::mutex::scoped_lock lock( _mutexConvertersQueue );
		if(!_convertersQueue.empty())
		{
		    size_t converterIndex = _convertersQueue.top()._converterIndex;
		    Converter::Converter& converter = _converters[converterIndex];
		    ros::Time schedule = _convertersQueue.top()._schedule;
		    PublisherConstIterator publisherIterator = _publisherMap.find( converter.name() );
		    if ( _publishEnabled &&  publisherIterator != _publisherMap.end() && publisherIterator->second.isSubscribed() )
		    {
			actions.push_back(MessageAction::PUBLISH);
		    }

		    if ( actions.size()>0 )
		    {
			converter.callAll( actions );
		    }

		    ros::Duration d( schedule - ros::Time::now() );
		    if ( d > ros::Duration(0))
		    {
			d.sleep();
		    }

		    _convertersQueue.pop();
		    if ( converter.getFrequency() != 0 )
		    {
			_convertersQueue.push(Helpers::ScheduledConverter(schedule + ros::Duration(1.0f / converter.getFrequency()), converterIndex));
		    }
		}
		else
		{
		    ros::Duration(1).sleep();
		}
	    }
	    if ( _publishEnabled )
	    {
		ros::spinOnce();
	    }
	}
    }

    void RobotToolkit::startRosLoop()
    {
	if (_mainThread.get_id() ==  boost::thread::id())
	    _mainThread = boost::thread( &RobotToolkit::rosLoop, this );
	_isRosLoopEnabled = true;
    }

    void RobotToolkit::stopRosLoop()
    {
	_isRosLoopEnabled = false;
	if (_mainThread.get_id() !=  boost::thread::id())
	    _mainThread.join();
    }

    void RobotToolkit::setMasterURINet(const std::string& uri, const std::string& networkInterface)
    {
	boost::mutex::scoped_lock lock( _mutexConvertersQueue );
	{
	    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] " << "Going to reset the node Handle" << RESETCOLOR << std::endl;
	    _nodeHandlerPtr.reset();
	    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] " << "Nodehandle reset " << std::endl;
	    Sinfonia::RosEnvironment::setMasterURI( uri, networkInterface );
	    _nodeHandlerPtr.reset( new ros::NodeHandle("~") );
	}
	if(_converters.empty())
	{
	    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] " << "Going to register converters" << RESETCOLOR << std::endl;
	    registerDefaultConverter();
	    registerDefaultSubscriber();

	}
	else
	{
	    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] " << "NOT going to re-register the converters" << std::endl;
	    typedef std::map< std::string, Publisher::Publisher > publisherMap;
	    resetService(* _nodeHandlerPtr);
	}
	startPublishing();

    }


    void RobotToolkit::startPublishing()
    {
	_publishEnabled = true;
    }

    void RobotToolkit::startInitialTopics()
    {
	// Poner aqui lo que se quiere iniciar por default
	/*scheduleConverter("tf", 50.0f);
	scheduleConverter("depth_to_laser", 20.0f);
	scheduleConverter("odom", 10.0f);
	scheduleConverter("merged_laser", 20.0f);
	scheduleConverter("laser", 20.0f);

	startSubscriber("cmd_vel");*/
	startSubscriber("special_settings");
	startRosLoop();
	openSharedMemory();
	std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] " << "Robot Toolkit Ready !!!" << std::endl;

    }


    void RobotToolkit::stopService()
    {
	sendSharedMemory("PepperHeadSharedMemory", 0);
	/*
	if(_shmfdPepperHead > 0)
	    shm_unlink("PepperHeadSharedMemory");
	*/

	sendSharedMemory("Depth2LaserSharedMemory", 0);
	/*
	 * if(_shmfdDepth2Laser > 0)
	    shm_unlink("Depth2LaserSharedMemory");
	 */

	sendSharedMemory("PepperLocalizerSharedMemory", 0);
	/*
	 * if(_shmfdPepperLocalizer > 0)
	    shm_unlink("PepperLocalizerSharedMemory");
	    */

	sendSharedMemory("PepperPlannerSharedMemory", 0);
	/*
	 * if(_shmfdPepperPlanner > 0)
	    shm_unlink("PepperPlannerSharedMemory");
	*/


	int frontFaceDetectorIndex = getConverterIndex("front_camera_face_detector");
	int bottomFaceDetectorIndex = getConverterIndex("bottom_camera_face_detector");

	_eventMap.find("mic")->second.shutdownEvents();
	_converters[frontFaceDetectorIndex].shutdown();
	_converters[bottomFaceDetectorIndex].shutdown();

	stopRosLoop();
	_eventMap.clear();
	_converters.clear();
	_subscribers.clear();


    }

    void RobotToolkit::registerDefaultConverter()
    {
	_tf2Buffer.reset<tf2_ros::Buffer>( new tf2_ros::Buffer() );
	_tf2Buffer->setUsingDedicatedThread(true);

	boost::shared_ptr<Publisher::TfPublisher> tfPublisher = boost::make_shared<Publisher::TfPublisher>("/tf");
	boost::shared_ptr<Converter::TfConverter> tfConverter = boost::make_shared<Converter::TfConverter>( "tf", 50, _tf2Buffer, _sessionPtr );
	if(_publishOdom)
		tfConverter->setPublishOdom();
	tfConverter->registerCallback( MessageAction::PUBLISH, boost::bind(&Publisher::TfPublisher::publish, tfPublisher, _1) );
	registerGroup( tfConverter, tfPublisher);

	boost::shared_ptr<Publisher::OdomPublisher > odomPublisher;
	if(_publishOdom)
		odomPublisher = boost::make_shared<Publisher::OdomPublisher>("/odom");
	else
		odomPublisher = boost::make_shared<Publisher::OdomPublisher>("/odom_wheels");
	boost::shared_ptr<Converter::OdomConverter> odomConverter = boost::make_shared<Converter::OdomConverter>("odom", 10, _sessionPtr );
	odomConverter->registerCallback( MessageAction::PUBLISH, boost::bind(&Publisher::OdomPublisher::publish, odomPublisher, _1) );
	registerGroup( odomConverter, odomPublisher);

	boost::shared_ptr<Publisher::LaserPublisher> laserPublisher = boost::make_shared<Publisher::LaserPublisher>("/laser");
	boost::shared_ptr<Converter::LaserConverter> laserConverter = boost::make_shared<Converter::LaserConverter>( "laser", 10, _sessionPtr );
	laserConverter->registerCallback( MessageAction::PUBLISH, boost::bind(&Publisher::LaserPublisher::publish, laserPublisher, _1) );
	registerGroup( laserConverter, laserPublisher);

	boost::shared_ptr<Publisher::LaserPublisher> mergedLaserPublisher = boost::make_shared<Publisher::LaserPublisher>("/merged_laser");
	boost::shared_ptr<Converter::LaserMergedConverter> mergedLaserConverter = boost::make_shared<Converter::LaserMergedConverter>( "merged_laser", 10, _sessionPtr );
	mergedLaserConverter->registerCallback( MessageAction::PUBLISH, boost::bind(&Publisher::LaserPublisher::publish, mergedLaserPublisher, _1) );
	registerGroup( mergedLaserConverter, mergedLaserPublisher);

	boost::shared_ptr<Publisher::LaserPublisher> depthToLaserPublisher = boost::make_shared<Publisher::LaserPublisher>("/depth_to_laser");
	//boost::shared_ptr<Converter::DepthToLaserConverter> depthToLaserConverter = boost::make_shared<Converter::DepthToLaserConverter>( "depth_to_laser", 10, _sessionPtr, Helpers::VisionHelpers::kQVGA);
	boost::shared_ptr<Converter::NaoqiDepth2LaserConverter> depthToLaserConverter = boost::make_shared<Converter::NaoqiDepth2LaserConverter>( "depth_to_laser", 10, _sessionPtr);
	depthToLaserConverter->registerCallback( MessageAction::PUBLISH, boost::bind(&Publisher::LaserPublisher::publish, depthToLaserPublisher, _1) );
	registerGroup( depthToLaserConverter, depthToLaserPublisher);

	boost::shared_ptr<Publisher::CameraPublisher> frontCameraPublisher = boost::make_shared<Publisher::CameraPublisher>("camera/front/image_raw");
	boost::shared_ptr<Converter::CameraConverter> frontCameraConverter = boost::make_shared<Converter::CameraConverter>("front_camera", 10, _sessionPtr, Helpers::VisionHelpers::kTopCamera, Helpers::VisionHelpers::kQVGA, Helpers::VisionHelpers::kRGBColorSpace);
	frontCameraConverter->registerCallback( MessageAction::PUBLISH, boost::bind(&Publisher::CameraPublisher::publish, frontCameraPublisher, _1, _2) );
	registerGroup( frontCameraConverter, frontCameraPublisher);

	boost::shared_ptr<Publisher::CameraPublisher> bottomCameraPublisher = boost::make_shared<Publisher::CameraPublisher>("camera/bottom/image_raw");
	boost::shared_ptr<Converter::CameraConverter> bottomCameraConverter = boost::make_shared<Converter::CameraConverter>("bottom_camera", 10, _sessionPtr, Helpers::VisionHelpers::kBottomCamera, Helpers::VisionHelpers::kQVGA, Helpers::VisionHelpers::kRGBColorSpace);
	bottomCameraConverter->registerCallback( MessageAction::PUBLISH, boost::bind(&Publisher::CameraPublisher::publish, bottomCameraPublisher, _1, _2) );
	registerGroup( bottomCameraConverter, bottomCameraPublisher);

	boost::shared_ptr<Publisher::CameraPublisher> depthCameraPublisher = boost::make_shared<Publisher::CameraPublisher>("camera/depth/image_raw");
	boost::shared_ptr<Converter::CameraConverter> depthCameraConverter = boost::make_shared<Converter::CameraConverter>("depth_camera", 10, _sessionPtr, Helpers::VisionHelpers::kDepthCamera, Helpers::VisionHelpers::kQVGA, Helpers::VisionHelpers::kRawDepthColorSpace);
	depthCameraConverter->registerCallback( MessageAction::PUBLISH, boost::bind(&Publisher::CameraPublisher::publish, depthCameraPublisher, _1, _2) );
	registerGroup( depthCameraConverter, depthCameraPublisher);


	boost::shared_ptr<Publisher::FacePublisher> topCameraFaceDetectorPublisher = boost::make_shared<Publisher::FacePublisher>("/face_publisher/front_camera");
	boost::shared_ptr<Sinfonia::Converter::FaceDetector> faceDetectorTopCamera = boost::make_shared<Sinfonia::Converter::FaceDetector>("front_camera_face_detector", 10, _sessionPtr, Helpers::VisionHelpers::kTopCamera, Helpers::VisionHelpers::kQVGA, Helpers::VisionHelpers::kRGBColorSpace);
	faceDetectorTopCamera->registerCallback( MessageAction::PUBLISH, boost::bind(&Publisher::FacePublisher::publish, topCameraFaceDetectorPublisher, _1) );
	registerGroup( faceDetectorTopCamera, topCameraFaceDetectorPublisher);

	boost::shared_ptr<Publisher::FacePublisher> bottomCameraFaceDetectorPublisher = boost::make_shared<Publisher::FacePublisher>("/face_publisher/bottom_camera");
	boost::shared_ptr<Sinfonia::Converter::FaceDetector> faceDetectorBottomCamera = boost::make_shared<Sinfonia::Converter::FaceDetector>("bottom_camera_face_detector", 10, _sessionPtr, Helpers::VisionHelpers::kBottomCamera, Helpers::VisionHelpers::kQVGA, Helpers::VisionHelpers::kRGBColorSpace);
	faceDetectorBottomCamera->registerCallback( MessageAction::PUBLISH, boost::bind(&Publisher::FacePublisher::publish, bottomCameraFaceDetectorPublisher, _1) );
	registerGroup( faceDetectorBottomCamera, bottomCameraFaceDetectorPublisher);


	boost::shared_ptr< Sinfonia::MicEventRegister > audioEventRegister = boost::make_shared<Sinfonia::MicEventRegister>("mic", 0, _sessionPtr);
	insertEventConverter("mic", audioEventRegister);

	boost::shared_ptr< Sinfonia::MicLocalizationEvent > micLocalizationEvent = boost::make_shared<Sinfonia::MicLocalizationEvent>("miclocalization", 0, _sessionPtr);
	insertEventConverter("miclocalization", micLocalizationEvent);

	boost::shared_ptr< Sinfonia::MiscToolsEvents::TouchEvent > touchEvent = boost::make_shared<Sinfonia::MiscToolsEvents::TouchEvent>("touch", 0, _sessionPtr);
	insertEventConverter("touch", touchEvent);

	std::vector<std::string> sonarTopics;
	sonarTopics.push_back("/sonar/front");
	sonarTopics.push_back("/sonar/back");

	boost::shared_ptr<Publisher::SonarPublisher> sonarPublisher = boost::make_shared<Publisher::SonarPublisher>(sonarTopics);
	boost::shared_ptr<Converter::SonarConverter> sonarConverter = boost::make_shared<Converter::SonarConverter>( "sonar", 10, _sessionPtr);
	sonarConverter->registerCallback( MessageAction::PUBLISH, boost::bind(&Publisher::SonarPublisher::publish, sonarPublisher, _1) );
	registerGroup( sonarConverter, sonarPublisher);

	boost::shared_ptr<Publisher::PathPublisher> pathPublisher = boost::make_shared<Publisher::PathPublisher>("/navigation/path");
	boost::shared_ptr<Converter::PathConverter> pathConverter = boost::make_shared<Converter::PathConverter>( "navigation_path", 10, _sessionPtr);
	pathConverter->registerCallback( MessageAction::PUBLISH, boost::bind(&Publisher::PathPublisher::publish, pathPublisher, _1) );
	registerGroup( pathConverter, pathPublisher);


	boost::shared_ptr<Publisher::RobotPosePublisher > robotPosePublisher = boost::make_shared<Publisher::RobotPosePublisher>("/navigation/robot_pose_publisher");
	boost::shared_ptr<Converter::RobotPoseConverter> robotPoseConverter = boost::make_shared<Converter::RobotPoseConverter>( "navigation_robot_pose", 10, _sessionPtr);
	robotPoseConverter->registerCallback( MessageAction::PUBLISH, boost::bind(&Publisher::RobotPosePublisher::publish, robotPosePublisher, _1) );
	registerGroup( robotPoseConverter, robotPosePublisher);



	boost::shared_ptr< Sinfonia::Navigation::ResultEvent > navigationResultEvent = boost::make_shared<Sinfonia::Navigation::ResultEvent>("navigation_result", 0, _sessionPtr);
	insertEventConverter("navigation_result", navigationResultEvent);

	printRegisteredConverters();

    }

    void RobotToolkit::registerGroup(Converter::Converter converter, Publisher::Publisher publisher)
    {
	registerConverter(converter);
	registerPublisher(converter.name(), publisher);
    }

    void RobotToolkit::registerConverter(Converter::Converter& converter)
    {
	boost::mutex::scoped_lock lock( _mutexConvertersQueue );
	int convIndex = _converters.size();
	_converters.push_back( converter );
	converter.reset();

    }

    void RobotToolkit::registerPublisher(const std::string& converterName, Publisher::Publisher& publisher)
    {
	_publisherMap.insert( std::map<std::string, Publisher::Publisher>::value_type(converterName, publisher) );
    }

    void RobotToolkit::printRegisteredConverters()
    {
	for( int i=0; i<_converters.size(); i++ )
	{
	    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] " << "Registered /" << _converters[i].name() << " converter. " << std::endl;
	}
    }

    void RobotToolkit::registerDefaultSubscriber()
    {
	if (!_subscribers.empty())
	    return;
	registerSubscriber(boost::make_shared<Subscriber::CmdVelSubscriber>("cmd_vel", "/cmd_vel", _sessionPtr));
	registerSubscriber(boost::make_shared<Subscriber::SpeechSubscriber>("speech", "/speech", _sessionPtr));
	registerSubscriber(boost::make_shared<Subscriber::MoveToSubscriber>("moveto", "/move_base_simple/goal", _sessionPtr, _tf2Buffer));
	registerSubscriber(boost::make_shared<Subscriber::AnimationSubscriber>("animation", "/animations", _sessionPtr));
	registerSubscriber(boost::make_shared<Subscriber::SetAnglesSubscriber>("set_angles", "/set_angles", _sessionPtr));
	registerSubscriber(boost::make_shared<Subscriber::LedsSubscriber>("leds", "/leds", _sessionPtr));
	registerSubscriber(boost::make_shared<Subscriber::NavigationGoalSubscriber>("navigation_goal", "/navigation/goal", _sessionPtr));
	registerSubscriber(boost::make_shared<Subscriber::RobotPoseSubscriber>("navigation_robot_pose", "/navigation/robot_pose_subscriber", _sessionPtr));
	registerSubscriber(boost::make_shared<Subscriber::SpecialSettingsSubscriber>("special_settings", "/special_settings", _sessionPtr));
	registerSubscriber(boost::make_shared<Subscriber::FreeZoneSubscriber>("free_zone", "/free_zone", _sessionPtr));
    }

    void RobotToolkit::registerSubscriber(Subscriber::Subscriber subscriber)
    {
	std::vector<Subscriber::Subscriber>::iterator it;
	it = std::find( _subscribers.begin(), _subscribers.end(), subscriber );
	size_t subIndex = 0;

	if (it == _subscribers.end() )
	{
	    subIndex = _subscribers.size();
	    _subscribers.push_back( subscriber );
	    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] " << "Registered /" << subscriber.name() << " subscriber."<< std::endl;
	}

	else
	{
	    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] " << "re-initialized existing subscriber:\t" << it->name() << std::endl;
	}
    }

    void RobotToolkit::scheduleConverter(std::string converterName, float converterFrequency)
    {
	boost::mutex::scoped_lock lock( _mutexConvertersQueue );
	std::priority_queue<Helpers::ScheduledConverter> auxiliarQueue = _convertersQueue;
	bool exist = false;
	for( int i=0; i<_convertersQueue.size(); i++)
	{
	    int converterIndex = auxiliarQueue.top()._converterIndex;
	    if(_converters[converterIndex].name() == converterName)
	    {
		_converters[converterIndex].setFrequency(converterFrequency);
		exist = true;
		break;
	    }
	    auxiliarQueue.pop();

	}
	if (!exist)
	{
	    for( int i=0; i<_converters.size(); i++ )
	    {
		if(_converters[i].name() == converterName)
		{
		    _converters[i].setFrequency(converterFrequency);
		    _convertersQueue.push(Helpers::ScheduledConverter(ros::Time::now(), i));
		}
	    }
	    typedef std::map< std::string, Publisher::Publisher > publisherMap;
	    for_each( publisherMap::value_type &pub, _publisherMap )
	    {
		if (pub.first.c_str() == converterName)
		{
		    pub.second.reset(*_nodeHandlerPtr);
		}
	    }
	}
    }

    void RobotToolkit::unscheduleConverter(std::string converterName)
    {
	boost::mutex::scoped_lock lock( _mutexConvertersQueue );
	std::vector< Helpers::ScheduledConverter > auxiliarQueue;
	size_t converterIndex;
	typedef std::map< std::string, Publisher::Publisher > publisherMap;
	int queueSize = _convertersQueue.size();
	for( int i=0; i<queueSize; i++)
	{
	    converterIndex = _convertersQueue.top()._converterIndex;
	    Converter::Converter& converter = _converters[converterIndex];
	    if(converter.name() != converterName)
	    {
		auxiliarQueue.push_back(_convertersQueue.top());
	    }
	    _convertersQueue.pop();
	}
	for( int i = 0; i < auxiliarQueue.size(); i++)
	{
	    _convertersQueue.push(auxiliarQueue[i]);
	}
	for_each( publisherMap::value_type &pub, _publisherMap )
	{
	    if (pub.first.c_str() == converterName)
	    {
		pub.second.shutdown();
	    }
	}
    }

    void RobotToolkit::startSubscriber(std::string subscriberName)
    {
	for_each( Subscriber::Subscriber& sub, _subscribers )
	{
	    if(sub.name() == subscriberName)
	    {
		sub.reset( *_nodeHandlerPtr );
	    }
	}
    }

    void RobotToolkit::stopSubscriber(std::string subscriberName)
    {
	for_each( Subscriber::Subscriber& sub, _subscribers )
	{
	    if(sub.name() == subscriberName)
	    {
		sub.shutdown();
	    }
	}
    }

    void RobotToolkit::resetService(ros::NodeHandle& nodeHandle)
    {
	_navigationToolsService = nodeHandle.advertiseService("/robot_toolkit/navigation_tools_srv" , &RobotToolkit::navigationToolsCallback, this);
	_visionToolsService = nodeHandle.advertiseService("/robot_toolkit/vision_tools_srv" , &RobotToolkit::visionToolsCallback, this);
	_audioToolsService = nodeHandle.advertiseService("/robot_toolkit/audio_tools_srv" , &RobotToolkit::audioToolsCallback , this);
	_motionToolsService = nodeHandle.advertiseService("/robot_toolkit/motion_tools_srv" , &RobotToolkit::motionToolsCallback , this);
	_miscToolsService = nodeHandle.advertiseService("/robot_toolkit/misc_tools_srv" , &RobotToolkit::miscToolsCallback , this);
  _speechRecognitionService = nodeHandle.advertiseService("/robot_toolkit/speech_recognition_srv" , &RobotToolkit::speechRecognitionCallback , this);
    }

    bool RobotToolkit::navigationToolsCallback( robot_toolkit_msgs::navigation_tools_srv::Request& request, robot_toolkit_msgs::navigation_tools_srv::Response& response )
    {
	std::string responseMessage;

	if( request.data.command == "enable_mapper" )
	{
	    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] Starting navigation "   << std::endl;
	    scheduleConverter("tf", 50.0f);
	    scheduleConverter("laser", 10.0f);
	    scheduleConverter("depth_to_laser", 10.0f);
            scheduleConverter("odom", 10.0f);
	    scheduleConverter("merged_laser", 10.0f);

	    startSubscriber("cmd_vel");

	    sendSharedMemory("Depth2LaserSharedMemory", 1);
            sendSharedMemory("PepperHeadSharedMemory", 1);


	    responseMessage = "Functionalities started: tf, laser, depth to laser, odom, merged_laser, Depth2LaserSharedMemory, PepperHeadSharedMemory";
	    std::cout << BOLDYELLOW << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
	}
	else if( request.data.command == "disable_mapper" )
	{
	    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] Stopping navigation "   << std::endl;
	    unscheduleConverter("tf");
	    unscheduleConverter("laser");
	    unscheduleConverter("depth_to_laser");
            unscheduleConverter("odom");
	    unscheduleConverter("merged_laser");
	    stopSubscriber("cmd_vel");

	    sendSharedMemory("Depth2LaserSharedMemory", 0);
            sendSharedMemory("PepperHeadSharedMemory", 0);


	    responseMessage = "Functionalities stopped: tf, laser, depth to laser, odom, merged_laser, Depth2LaserSharedMemory, PepperHeadSharedMemory";
	    std::cout << BOLDYELLOW << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
	}

	else if( request.data.command == "enable_navigate" )
	{
	    //std::cout << "Entramos aqui " << std::endl;

	    sendSharedMemory("PepperHeadSharedMemory", 1);
	    sendSharedMemory("Depth2LaserSharedMemory", 1);
	    sendSharedMemory("PepperLocalizerSharedMemory", 1);
	    sendSharedMemory("PepperPlannerSharedMemory", 1);

	    startSubscriber("navigation_goal");
	    startSubscriber("navigation_robot_pose");
	    startSubscriber("free_zone");
	    _eventMap.find("navigation_result")->second.startProcess();
	    _eventMap.find("navigation_result")->second.resetPublisher(*_nodeHandlerPtr);
	    scheduleConverter("navigation_path", 10.0f);
	    scheduleConverter("navigation_robot_pose", 10.0f);

	    responseMessage = "Functionalities started: pepper_head, localizer, planner, navigation_result ,depth_to_laser@10Hz, navigation_goal, navigation_robot_pose, navigation_result, navigation_path, navigation_robot_pose, free_zone";
	    std::cout << BOLDYELLOW << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
	}

	else if( request.data.command == "disable_navigate" )
	{
	    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] Stopping navigation "   << std::endl;
	    sendSharedMemory("PepperHeadSharedMemory", 0);
	    sendSharedMemory("Depth2LaserSharedMemory", 0);
	    sendSharedMemory("PepperLocalizerSharedMemory", 0);
	    sendSharedMemory("PepperPlannerSharedMemory", 0);

	    stopSubscriber("navigation_goal");
	    stopSubscriber("navigation_robot_pose");
	    stopSubscriber("free_zone");
	    _eventMap.find("navigation_result")->second.stopProcess();
	    _eventMap.find("navigation_result")->second.shutdownPublisher();
	    unscheduleConverter("navigation_path");
	    unscheduleConverter("navigation_robot_pose");

	    responseMessage = "Functionalities stopped: pepper_head, localizer, planner, navigation_result ,depth_to_laser@10Hz, navigation_goal, navigation_robot_pose, navigation_result, navigation_path, navigation_robot_pose, free_zone";
	    std::cout << BOLDYELLOW << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
	}
	else if( request.data.command == "enable_all" )
	{
	    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "]" << " Starting Navigation Tools " << RESETCOLOR  << std::endl;
	    scheduleConverter("tf", 50.0f);
	    scheduleConverter("odom", 10.0f);

	    scheduleConverter("laser", 10.0f);

	    startSubscriber("navigation_goal");
	    startSubscriber("navigation_robot_pose");
	    startSubscriber("free_zone");
	    _eventMap.find("navigation_result")->second.startProcess();
	    _eventMap.find("navigation_result")->second.resetPublisher(*_nodeHandlerPtr);
	    scheduleConverter("navigation_path", 10.0f);
	    scheduleConverter("navigation_robot_pose", 10.0f);
	    scheduleConverter("depth_to_laser", 10.0f);
	    sendSharedMemory("Depth2LaserSharedMemory", 1);



	    int subscriberIndex = getSubscriberIndex("cmd_vel");

	    if( subscriberIndex != -1)
	    {
		_subscribers[subscriberIndex].setDefaultParameters();
		startSubscriber("cmd_vel");
	    }
	    startSubscriber("moveto");
	    responseMessage = "Functionalities started: tf@50Hz, odom@10Hz, laser@10Hz, depth_to_laser@10Hz, cmd_vel, move_to, navigation_goal, navigation_robot_pose, navigation_result, navigation_path, navigation_robot_pose, free_zone";
	    std::cout << BOLDYELLOW << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
	}
	else if( request.data.command == "disable_all" )
	{
	    std::cout << BOLDBLUE << "[" << ros::Time::now().toSec() << "]" << " Stopping Navigation Tools " << RESETCOLOR  << std::endl;
	    unscheduleConverter("tf");
	    unscheduleConverter("odom");
	    unscheduleConverter("laser");
	    unscheduleConverter("depth_to_laser");
	    stopSubscriber("cmd_vel");
	    stopSubscriber("moveto");
	    sendSharedMemory("Depth2LaserSharedMemory", 0);
	    stopSubscriber("navigation_goal");
	    stopSubscriber("navigation_robot_pose");
	    stopSubscriber("free_zone");
	    _eventMap.find("navigation_result")->second.stopProcess();
	    _eventMap.find("navigation_result")->second.shutdownPublisher();
	    unscheduleConverter("navigation_path");
	    unscheduleConverter("navigation_robot_pose");


	    responseMessage = "Functionalities stopped: tf, odom, laser, depth_to_laser, cmd_vel, move_to, navigation_goal, navigation_robot_pose, navigation_result, navigation_path, navigation_robot_pose";
	    std::cout << BOLDYELLOW << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
	}
	else if( request.data.command == "custom" )
	{
	    std::cout << BOLDMAGENTA << "[" << ros::Time::now().toSec() << "]" << " Setting up navigation_tools " << RESETCOLOR  << std::endl;
	    std::string startedFunctionalities = "Functionalities started: ";
	    std::string stoppedFunctionalities = "Functionalities stopped: ";
	    responseMessage = "topics";
	    if( request.data.tf_enable )
	    {
		scheduleConverter("tf", request.data.tf_frequency);
		startedFunctionalities += "tf@" + boost::lexical_cast<std::string>(request.data.tf_frequency) + "Hz, ";
	    }
	    else
	    {
		unscheduleConverter("tf");
		stoppedFunctionalities += "tf, ";
	    }

	    if( request.data.odom_enable )
	    {
		startedFunctionalities += "odom@" + boost::lexical_cast<std::string>(request.data.odom_frequency) + "Hz, ";
		scheduleConverter("odom", request.data.odom_frequency);
	    }
	    else
	    {
		unscheduleConverter("odom");
		stoppedFunctionalities += "odom, ";
	    }

	    if( request.data.laser_enable )
	    {
		startedFunctionalities += "laser@" + boost::lexical_cast<std::string>(request.data.laser_frequency) + "Hz, ";
		scheduleConverter("laser", request.data.laser_frequency);
	    }
	    else
	    {
		unscheduleConverter("laser");
		stoppedFunctionalities += "laser, ";
	    }
	    //////////////////////////////////////////////////////
	    if( request.data.goal_enable )
	    {
		startedFunctionalities += "navigation_goal";
		startSubscriber("navigation_goal");
	    }
	    else
	    {
		stopSubscriber("navigation_goal");
		stoppedFunctionalities += "navigation_goal, ";
	    }
	    //---------------
	    if( request.data.robot_pose_suscriber_enable )
	    {
		startedFunctionalities += "navigation_robot_pose";
		startSubscriber("navigation_robot_pose");
	    }
	    else
	    {
		stopSubscriber("navigation_robot_pose");
		stoppedFunctionalities += "navigation_robot_pose, ";
	    }
	    //-----------------------------
	    if( request.data.path_enable )
	    {
		startedFunctionalities += "navigation_path" + boost::lexical_cast<std::string>(request.data.path_frequency) + "Hz, ";
		scheduleConverter("navigation_path", request.data.path_frequency);
	    }
	    else
	    {
		unscheduleConverter("navigation_path");
		stoppedFunctionalities += "navigation_path, ";
	    }
	    //-----
	    if( request.data.robot_pose_publisher_enable )
	    {
		startedFunctionalities += "navigation_robot_pose" + boost::lexical_cast<std::string>(request.data.robot_pose_publisher_frequency) + "Hz, ";
		scheduleConverter("navigation_robot_pose", request.data.robot_pose_publisher_frequency);
	    }
	    else
	    {
		unscheduleConverter("navigation_robot_pose");
		stoppedFunctionalities += "navigation_robot_pose, ";
	    }
	    //--------
	    if( request.data.result_enable )
	    {
		startedFunctionalities += "navigation_result ";
		_eventMap.find("navigation_result")->second.startProcess();
		_eventMap.find("navigation_result")->second.resetPublisher(*_nodeHandlerPtr);
	    }
	    else
	    {
		_eventMap.find("navigation_result")->second.stopProcess();
		_eventMap.find("navigation_result")->second.shutdownPublisher();
		stoppedFunctionalities += "navigation_result, ";
	    }
	    /////////////////////////
	    if( request.data.depth_to_laser_enable )
	    {
		int converterIndex = getConverterIndex("depth_to_laser");
		if( converterIndex != -1 )
		{
		    std::vector<float> config;
		    config.push_back(request.data.depth_to_laser_parameters.resolution);
		    std::vector<float> parameters;
		    parameters.push_back(request.data.depth_to_laser_parameters.scan_time);
		    parameters.push_back(request.data.depth_to_laser_parameters.range_min);
		    parameters.push_back(request.data.depth_to_laser_parameters.range_max);
		    parameters.push_back(request.data.depth_to_laser_parameters.scan_height);

		    _converters[converterIndex].setConfig(config);
		    _converters[converterIndex].setParameters(parameters);
		    _converters[converterIndex].reset();
		    scheduleConverter("depth_to_laser", 10.0f);
		    sendSharedMemory("Depth2LaserSharedMemory", 1);
		}
		startedFunctionalities += "depth_to_laser@10Hz";

	    }
	    else
	    {
		unscheduleConverter("depth_to_laser");
		stoppedFunctionalities += "depth_to_laser, ";
		sendSharedMemory("Depth2LaserSharedMemory", 0);
	    }

	    if( request.data.cmd_vel_enable )
	    {
		int securityTime;
		std::vector<float> parameters;
		if( request.data.security_timer <= 0 )
		{
		    securityTime = -1;
		}
		else
		{
		    securityTime = request.data.security_timer;
		}
		parameters.push_back(securityTime);
		int subscriberIndex = getSubscriberIndex("cmd_vel");
		_subscribers[subscriberIndex].setParameters(parameters);
		startSubscriber("cmd_vel");
		startedFunctionalities += "cmd_vel, ";
	    }
	    else
	    {
		stopSubscriber("cmd_vel");
		stoppedFunctionalities += "cmd_vel, ";
	    }

	    if( request.data.move_base_enable )
	    {
		startSubscriber("moveto");
		startedFunctionalities += "move_to ";
	    }
	    else
	    {
		stopSubscriber("moveto");
		stoppedFunctionalities += "move_to ";
	    }
	    if( request.data.free_zone_enable )
	    {
		startSubscriber("free_zone");
		startedFunctionalities += "free_zone ";
	    }
	    else
	    {
		stopSubscriber("free_zone");
		stoppedFunctionalities += "free_zone ";
	    }
	    std::cout << BOLDYELLOW << "[" << ros::Time::now().toSec() << "] " << startedFunctionalities << RESETCOLOR  << std::endl;
	    std::cout << BOLDYELLOW << "[" << ros::Time::now().toSec() << "] " << stoppedFunctionalities << RESETCOLOR  << std::endl;
	    responseMessage = startedFunctionalities + stoppedFunctionalities;
	}
	else
	{
	    responseMessage = "ERROR: unkown command in navigation_tools service";
	    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
	}
	response.result =  responseMessage;
	return true;
    }

    bool RobotToolkit::visionToolsCallback(robot_toolkit_msgs::vision_tools_srv::Request& request, robot_toolkit_msgs::vision_tools_srv::Response& response)
    {
	std::string responseMessage;
	robot_toolkit_msgs::camera_parameters_msg currentParamsMessage;
	if( request.data.camera_name != "front_camera" && request.data.camera_name != "bottom_camera" && request.data.camera_name != "depth_camera" && request.data.camera_name != "front_camera_face_detector" && request.data.camera_name != "bottom_camera_face_detector")
	{
	    responseMessage = "ERROR: unknown camera name, possible values are: front_camera, bottom_camera, depth_camera";
	    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
	}
	else
	{
	    if( request.data.command == "enable" )
	    {
		int converterIndex = getConverterIndex(request.data.camera_name);
		if( converterIndex != -1 )
		{
		    std::vector<float> config;
		    float defaultFrequency;
		    if(request.data.camera_name == "front_camera_face_detector" || request.data.camera_name == "bottom_camera_face_detector")
		    {
			defaultFrequency = 30.0f;
			config.push_back(Helpers::VisionHelpers::kVGA);
		    }
		    else
		    {
			defaultFrequency = 10.0f;
			config.push_back(Helpers::VisionHelpers::kQVGA);
		    }
		    config.push_back(10);
		    if( request.data.camera_name == "depth_camera" )
		    {
			config.push_back(Helpers::VisionHelpers::kRawDepthColorSpace);
		    }
		    else
		    {
			config.push_back(Helpers::VisionHelpers::kRGBColorSpace);
		    }
		    _converters[converterIndex].setConfig(config);
		    _converters[converterIndex].reset();
		    if( request.data.camera_name != "depth_camera" )
		    {
			currentParamsMessage = toCameraParametersMsg(_converters[converterIndex].setAllParametersToDefault());
		    }
		    scheduleConverter(request.data.camera_name, defaultFrequency);
		    responseMessage = "Starting: " + request.data.camera_name + " with default parameters";
		    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "]" << " Starting: " << request.data.camera_name << " with default parameters" << RESETCOLOR  << std::endl;
		}
		else
		{
		    responseMessage = "ERROR: converter missing ";
		    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
		}
	    }
	    else if( request.data.command == "disable" )
	    {
		int converterIndex = getConverterIndex(request.data.camera_name);
		if( converterIndex != -1 )
		{
		    unscheduleConverter(request.data.camera_name);
		    responseMessage = "Shutting down: " + request.data.camera_name;
		    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "]" << " Shutting down: " << request.data.camera_name << RESETCOLOR  << std::endl;
		    if(request.data.camera_name == "front_camera_face_detector" || request.data.camera_name == "bottom_camera_face_detector")
		    {
			int converterIndex = getConverterIndex(request.data.camera_name);
			_converters[converterIndex].shutdown();
		    }
		}
		else
		{
		    responseMessage = "ERROR: converter missing ";
		    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;

		}
	    }
	    else if( request.data.command == "custom" )
	    {
		int converterIndex = getConverterIndex(request.data.camera_name);
		if( converterIndex != -1 )
		{
		    std::vector<int> config;
		    bool configOk = true;
		    if( request.data.camera_name != "depth_camera" )
		    {
			if(!(request.data.resolution <= Helpers::VisionHelpers::k16VGA || request.data.resolution == Helpers::VisionHelpers::kQQQVGA || request.data.resolution == Helpers::VisionHelpers::kQQQQVGA ))
			{
			    responseMessage = "ERROR: Bad resolution configuration for camera: " + request.data.camera_name;
			    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
			    configOk = false;
			}
			else if( request.data.frame_rate < 1 || request.data.frame_rate > 30 || ((request.data.resolution == Helpers::VisionHelpers::k4VGA) && (request.data.frame_rate != 1)) ||
			    ((request.data.resolution == Helpers::VisionHelpers::k16VGA) && (request.data.frame_rate != 1)))
			{
			    responseMessage = "ERROR: Bad frame rate configuration for camera: " + request.data.camera_name;
			    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
			    configOk = false;
			}
			else if( request.data.color_space > Helpers::VisionHelpers::kHSMixedColorSpace)
			{
			    responseMessage = "ERROR: Bad color space configuration for camera: " + request.data.camera_name;
			    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
			    configOk = false;
			}
		    }
		    else
		    {
			if(!(request.data.resolution <= Helpers::VisionHelpers::kQVGA || request.data.resolution == Helpers::VisionHelpers::kQQQVGA || request.data.resolution == Helpers::VisionHelpers::kQQQQVGA ))
			{
			    responseMessage = "ERROR: Bad resolution configuration for camera: " + request.data.camera_name;
			    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
			    configOk = false;
			}
			else if( request.data.frame_rate < 1 || request.data.frame_rate > 20 )
			{
			    responseMessage = "ERROR: Bad frame rate configuration for camera: " + request.data.camera_name;
			    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
			    configOk = false;
			}
			else if( !( request.data.color_space == Helpers::VisionHelpers::kYuvColorSpace ||  request.data.color_space == Helpers::VisionHelpers::kRGBColorSpace || request.data.color_space == Helpers::VisionHelpers::kDepthColorSpace ||
			    request.data.color_space == Helpers::VisionHelpers::kXYZColorSpace || request.data.color_space == Helpers::VisionHelpers::kDistanceColorSpace || request.data.color_space == Helpers::VisionHelpers::kRawDepthColorSpace ) )
			{
			    responseMessage = "ERROR: Bad color space configuration for camera: " + request.data.camera_name;
			    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
			    configOk = false;
			}
		    }
		    if(configOk)
		    {
			std::vector<float> config;
			config.push_back(request.data.resolution);
			config.push_back(request.data.frame_rate);
			config.push_back(request.data.color_space);
			_converters[converterIndex].setConfig(config);
			_converters[converterIndex].reset();
			if( request.data.camera_name != "depth_camera" )
			{
			    currentParamsMessage = toCameraParametersMsg(_converters[converterIndex].setAllParametersToDefault());
			}
			scheduleConverter(request.data.camera_name, request.data.frame_rate);
			responseMessage = " Starting: " + request.data.camera_name + " with custom configuration and parameters";
			std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "]" << responseMessage << RESETCOLOR  << std::endl;

		    }
		}
		else
		{
		    responseMessage = "ERROR: converter missing ";
		    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;

		}
	    }
	    else if( request.data.command == "set_parameters" && request.data.camera_name != "depth_camera")
	    {
		    int converterIndex = getConverterIndex(request.data.camera_name);
		    if( converterIndex != -1 )
		    {
			if(request.data.camera_name == "depth_camera")
			    request.data.camera_parameters.compress = false;
			currentParamsMessage = toCameraParametersMsg(_converters[converterIndex].setParameters(toVector(request.data.camera_parameters)));
			responseMessage = "Setting Parameters of " + request.data.camera_name;
			std::cout << BOLDMAGENTA << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
		    }
		    else
		    {
			responseMessage = "ERROR: converter missing ";
			std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;

		    }
	    }
	    else if( request.data.command == "get_parameters" && request.data.camera_name != "depth_camera")
	    {
		int converterIndex = getConverterIndex(request.data.camera_name);
		if( converterIndex != -1 )
		{
		    currentParamsMessage = toCameraParametersMsg(_converters[converterIndex].getParameters());
		    responseMessage = "Getting Parameters of " + request.data.camera_name;
		    std::cout << BOLDMAGENTA << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
		}
		else
		{
		    responseMessage = "ERROR: converter missing ";
		    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
		}
	    }
	    else
	    {
		responseMessage = "ERROR: unkown command in vision_tools service";
		std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
	    }
	}
	response.result = responseMessage;
	response.camera_parameters = currentParamsMessage;
	return true;
    }

    int RobotToolkit::getConverterIndex(std::string name)
    {
	for( int i=0; i<_converters.size(); i++ )
	{
	    if(_converters[i].name() == name)
	    {
		return i;
	    }
	}
	return -1;
    }

    robot_toolkit_msgs::camera_parameters_msg RobotToolkit::toCameraParametersMsg(std::vector< float > params)
    {
	robot_toolkit_msgs::camera_parameters_msg result;
	result.brightness = params[0];
	result.contrast = params[1];
	result.saturation = params[2];
	result.hue = params[3];
	result.horizontal_flip = params[4];
	result.vertical_flip = params[5];
	result.auto_exposition = params[6];
	result.auto_white_balance = params[7];
	result.auto_gain = params[8];
	result.gain = params[9];
	result.exposure = params[10];
	result.reset_camera_registers = params[11];
	result.blc_red_value = params[12];
	result.blc_green_value = params[13];
	result.blc_blue_value = params[14];
	result.resolution = params[15];
	result.fps = params[16];
	result.average_luminance = params[17];
	result.auto_focus = params[18];
	result.compress = params[19];
	result.compression_factor = params[20];
	return result;
    }

    std::vector< float > RobotToolkit::toVector(robot_toolkit_msgs::camera_parameters_msg params)
    {
	std::vector<float>  result;
	result.push_back(params.brightness);
	result.push_back(params.contrast);
	result.push_back(params.saturation);
	result.push_back(params.hue);
	result.push_back(params.horizontal_flip);
	result.push_back(params.vertical_flip);
	result.push_back(params.auto_exposition);
	result.push_back(params.auto_white_balance);
	result.push_back(params.auto_gain);
	result.push_back(params.gain);
	result.push_back(params.exposure);
	result.push_back(params.reset_camera_registers);
	result.push_back(params.blc_red_value);
	result.push_back(params.blc_green_value);
	result.push_back(params.blc_blue_value);
	result.push_back(params.resolution);
	result.push_back(params.fps);
	result.push_back(params.average_luminance);
	result.push_back(params.auto_focus);
	if(params.compress)
	    result.push_back(1);
	else
	    result.push_back(0);
	result.push_back(params.compression_factor);
	return result;
    }

    void RobotToolkit::insertEventConverter(const std::string& key, Event::Event event)
    {
	_eventMap.insert( std::map<std::string, Event::Event>::value_type(key, event));
    }

    bool RobotToolkit::audioToolsCallback(robot_toolkit_msgs::audio_tools_srv::Request& request, robot_toolkit_msgs::audio_tools_srv::Response& response)
    {
	std::string responseMessage;
	robot_toolkit_msgs::speech_parameters_msg currentSpeechParams;
	std::string startedFunctionalities = "Functionalities started:";
	std::string stoppedFunctionalities = "Functionalities stopped:";
	robot_toolkit_msgs::camera_parameters_msg currentParamsMessage;
	if( request.data.command != "enable" && request.data.command != "disable" && request.data.command != "custom" && request.data.command != "enable_mic" && request.data.command != "disable_mic" &&
	    request.data.command != "enable_tts" && request.data.command != "disable_tts" && request.data.command != "get_speech_params" && request.data.command != "set_speech_params" && request.data.command != "reset_speech_params" &&
	    request.data.command != "enable_localization" && request.data.command != "disable_localization" )
	{
	    responseMessage = "ERROR: unknown command, possible values are: enable, disable, custom, enable_mic, disable_mic, enable_tts, disable_tts, get_speech_params, set_speech_params, reset_speech_params";
	    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
	}
	else
	{
		if( request.data.command == "enable" || request.data.command == "enable_mic")
		{
		    _eventMap.find("mic")->second.setDefaultParameters();
		    _eventMap.find("mic")->second.startProcess();
		    _eventMap.find("mic")->second.resetPublisher(*_nodeHandlerPtr);
		    startedFunctionalities += " mic stream";
		    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] Starting mic stream" << RESETCOLOR  << std::endl;
		}

		if( request.data.command == "disable" || request.data.command == "disable_mic")
		{
		    _eventMap.find("mic")->second.stopProcess();
		    _eventMap.find("mic")->second.shutdownPublisher();
		    stoppedFunctionalities += " mic stream";
		    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] Stopping mic stream" << RESETCOLOR  << std::endl;
		}

		if( request.data.command == "custom" )
		{
		    if( (request.data.frequency == 48000 || request.data.frequency == 16000) && ( request.data.channels >= 0 && request.data.channels <= 4) )
		    {
			std::vector<int> parameters;
			parameters.push_back(request.data.frequency);
			parameters.push_back(request.data.channels);
			if(_eventMap.find("mic")->second.isStarted())
			{
			    _eventMap.find("mic")->second.stopProcess();
			    _eventMap.find("mic")->second.shutdownPublisher();
			};
			_eventMap.find("mic")->second.setParameters(parameters);
			_eventMap.find("mic")->second.startProcess();
			_eventMap.find("mic")->second.resetPublisher(*_nodeHandlerPtr);
			startedFunctionalities += " mic stream with custom parameters";
			std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] Starting mic stream with custom Parameters" << RESETCOLOR  << std::endl;
		    }
		    else
		    {
			responseMessage = "ERROR: invalid mic parameters";
			std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
			response.result = responseMessage;
			return true;
		    }
		}

		if( request.data.command == "enable" || request.data.command == "enable_tts")
		{
		    startSubscriber("speech");
		    startedFunctionalities += " tts";
		    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] Starting tts" << RESETCOLOR  << std::endl;
		    int subscriberIndex = getSubscriberIndex("speech");
		    currentSpeechParams = toSpeechParameters(_subscribers[subscriberIndex].getParameters());

		}

		if( request.data.command == "disable" || request.data.command == "disable_tts")
		{
		    stopSubscriber("speech");
		    stoppedFunctionalities += " tts";
		    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] Stopping tts" << RESETCOLOR  << std::endl;
		}

		if( request.data.command == "enable" || request.data.command == "enable_localization")
		{
		    _eventMap.find("miclocalization")->second.startProcess();
		    _eventMap.find("miclocalization")->second.resetPublisher(*_nodeHandlerPtr);
		    startedFunctionalities += " audio_localization";
		    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] Starting audio_localization" << RESETCOLOR  << std::endl;
		}

		if( request.data.command == "disable" || request.data.command == "disable_localization")
		{
		    _eventMap.find("miclocalization")->second.stopProcess();
		    _eventMap.find("miclocalization")->second.shutdownPublisher();
		    stoppedFunctionalities += " audio_localization";
		    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] Stopping audio_localization" << RESETCOLOR  << std::endl;
		}

		if( request.data.command == "get_speech_params")
		{
		    int subscriberIndex = getSubscriberIndex("speech");
		    currentSpeechParams = toSpeechParameters(_subscribers[subscriberIndex].getParameters());
		    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] Getting tts parameters" << RESETCOLOR  << std::endl;
		}

		if( request.data.command == "set_speech_params")
		{
		    int subscriberIndex = getSubscriberIndex("speech");
		    std::vector<float> speechParameters;
		    if( ( (request.data.speech_parameters.pitch_shift >= 1.0 && request.data.speech_parameters.pitch_shift <=4.0) || (request.data.speech_parameters.pitch_shift == 0.0) ) &&
			( (request.data.speech_parameters.double_voice >= 1.0 && request.data.speech_parameters.double_voice <=4.0) || (request.data.speech_parameters.double_voice == 0.0) )  &&
			  (request.data.speech_parameters.double_voice_level >= 0.0 && request.data.speech_parameters.double_voice_level <=4.0) &&
			  (request.data.speech_parameters.double_voice_time_shift >= 0.0 && request.data.speech_parameters.double_voice_time_shift <=0.5) &&
			  (request.data.speech_parameters.speed >= 50.0 && request.data.speech_parameters.speed <=400.0) )
		    {
			speechParameters.push_back(request.data.speech_parameters.pitch_shift);
			speechParameters.push_back(request.data.speech_parameters.double_voice);
			speechParameters.push_back(request.data.speech_parameters.double_voice_level);
			speechParameters.push_back(request.data.speech_parameters.double_voice_time_shift);
			speechParameters.push_back(request.data.speech_parameters.speed);
			_subscribers[subscriberIndex].setParameters(speechParameters);
			std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] Setting speech parameters" << RESETCOLOR  << std::endl;
			currentSpeechParams = toSpeechParameters(_subscribers[subscriberIndex].getParameters());
		    }
		    else
		    {
			responseMessage = "ERROR: invalid speech parameters";
			std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
			response.result = responseMessage;
			return true;
		    }
		}

		if( request.data.command == "reset_speech_params")
		{
		    int subscriberIndex = getSubscriberIndex("speech");
		    _subscribers[subscriberIndex].setDefaultParameters();
		    currentSpeechParams = toSpeechParameters(_subscribers[subscriberIndex].getParameters());
		    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] Resseting tts parameters" << RESETCOLOR  << std::endl;
		}

		responseMessage = startedFunctionalities + stoppedFunctionalities;
	}

	response.result = responseMessage;
	response.speech_parameters = currentSpeechParams;
	return true;
    }

    bool RobotToolkit::motionToolsCallback(robot_toolkit_msgs::motion_tools_srv::Request& request, robot_toolkit_msgs::motion_tools_srv::Response& response)
    {
	std::string responseMessage;
	std::string startedFunctionalities = "Functionalities started:";
	std::string stoppedFunctionalities = " Functionalities stopped:";
	robot_toolkit_msgs::camera_parameters_msg currentParamsMessage;
	if( request.data.command != "enable_all" && request.data.command != "disable_all" && request.data.command != "custom")
	{
	    responseMessage = "ERROR: unknown command: " + request.data.command +  " possible values are: enable_all, disable_all, custom";
	    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
	}
	else
	{
		if( request.data.command == "enable_all")
		{
		    startSubscriber("animation");
		    startSubscriber("set_angles");
		    responseMessage = "Starting Animations and Set Angles";
		    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;

		}

		if( request.data.command == "disable_all")
		{
		    stopSubscriber("animation");
		    stopSubscriber("set_angles");
		    responseMessage = "Stopping Animations and Set Angles";
		    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
		}

		else
		{
		    if(request.data.animation == "enable")
		    {
			startSubscriber("animation");
			startedFunctionalities += " Animation, ";
			std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] Starting animations" << RESETCOLOR  << std::endl;
		    }

		    else if(request.data.animation == "disable")
		    {
			stopSubscriber("animation");
			stoppedFunctionalities += " Animation, ";
			std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] Stopping animations" << RESETCOLOR  << std::endl;
		    }

		    if(request.data.set_angles == "enable")
		    {
			startSubscriber("set_angles");
			startedFunctionalities += " Set Angles, ";
			std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] Starting set angles" << RESETCOLOR  << std::endl;
		    }

		    else if(request.data.set_angles == "disable")
		    {
			stopSubscriber("set_angles");
			stoppedFunctionalities += " Set Angles, ";
			std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] Stopping set angles" << RESETCOLOR  << std::endl;
		    }

		    responseMessage = startedFunctionalities + stoppedFunctionalities;
		}

	}

	response.result = responseMessage;
	return true;
    }


    int RobotToolkit::getSubscriberIndex(std::string name)
    {
	for(int i=0; i<_subscribers.size(); i++)
	{
	    if(_subscribers[i].name() == name)
	    {
		return i;
	    }
	}
	return -1;
    }

    robot_toolkit_msgs::speech_parameters_msg RobotToolkit::toSpeechParameters(std::vector< float > params)
    {
	robot_toolkit_msgs::speech_parameters_msg result;

	result.pitch_shift = params[0];
	result.double_voice = params[1];
	result.double_voice_level = params[2];
	result.double_voice_time_shift = params[3];
	result.speed = params[4];

	return result;
    }

    bool RobotToolkit::speechRecognitionCallback(robot_toolkit_msgs::speech_recognition_srv::Request& request, robot_toolkit_msgs::speech_recognition_srv::Response& response)
    {
      //SpeechRecognitionEvent *speechRecognition = new SpeechRecognitionEvent("speech_recognition", 10.0f, _sessionPtr);
      std::vector<std::string> wordList;
      for(int i=0; i<request.words.size(); i++)
      {
        wordList.push_back(request.words[i]);
      }

      std::string wordRecognized  = _speechRecognition->startProcess(wordList, request.threshold);
      response.result = wordRecognized;
      return true;
    }

    bool RobotToolkit::miscToolsCallback(robot_toolkit_msgs::misc_tools_srv::Request& request, robot_toolkit_msgs::misc_tools_srv::Response& response)
    {
	std::string responseMessage;

	std::string startedFunctionalities = "Functionalities started:";
	std::string stoppedFunctionalities = " Functionalities stopped:";
	robot_toolkit_msgs::camera_parameters_msg currentParamsMessage;
	if( request.data.command != "enable_all" && request.data.command != "disable_all" && request.data.command != "custom" && request.data.command != "touch")
	{
	    responseMessage = "ERROR: unknown command: " + request.data.command +  " possible values are: enable_all, disable_all, custom";
	    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
	}
	else
	{
		if( request.data.command == "enable_all")
		{
		    startSubscriber("leds");
		    scheduleConverter("sonar", 50.0f);
		    _eventMap.find("touch")->second.startProcess();
		    _eventMap.find("touch")->second.resetPublisher(*_nodeHandlerPtr);
		    responseMessage = "Starting leds, touch and sonars";
		    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;

		}

		if( request.data.command == "disable_all")
		{
		    stopSubscriber("leds");
		    unscheduleConverter("sonar");
		    _eventMap.find("touch")->second.stopProcess();
		    _eventMap.find("touch")->second.shutdownPublisher();
		    responseMessage = "Stopping leds and sonars";
		    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
		}

		else
		{
		    if(request.data.leds == "enable")
		    {
			startSubscriber("leds");
			startedFunctionalities += " leds, ";
			std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] Starting leds" << RESETCOLOR  << std::endl;
		    }

		    else if(request.data.leds == "disable")
		    {
			stopSubscriber("leds");
			stoppedFunctionalities += " leds, ";
			std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] Stopping leds" << RESETCOLOR  << std::endl;
		    }

		    if(request.data.sonars == "enable")
		    {
			scheduleConverter("sonar", 50.0f);
			startedFunctionalities += " sonars, ";
			std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] Starting sonars" << RESETCOLOR  << std::endl;
		    }

		    else if(request.data.sonars == "disable")
		    {
			unscheduleConverter("sonar");
			stoppedFunctionalities += " sonars, ";
			std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] Stopping sonars" << RESETCOLOR  << std::endl;
		    }

		    if(request.data.touch == "enable")
		    {
			_eventMap.find("touch")->second.startProcess();
			_eventMap.find("touch")->second.resetPublisher(*_nodeHandlerPtr);
			startedFunctionalities += " touch, ";
			std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] Starting touch" << RESETCOLOR  << std::endl;
		    }

		    else if(request.data.touch == "disable")
		    {
			_eventMap.find("touch")->second.stopProcess();
			_eventMap.find("touch")->second.shutdownPublisher();
			stoppedFunctionalities += " touch, ";
			std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] Stopping touch" << RESETCOLOR  << std::endl;
		    }

		    responseMessage = startedFunctionalities + stoppedFunctionalities;
		}

	}

	response.result = responseMessage;
	return true;
    }

    void RobotToolkit::openSharedMemory()
    {
	/*const int SIZE = 1;
	_shmfdPepperHead = shm_open("PepperHeadSharedMemory", O_RDWR, 0666);
	if(_shmfdPepperHead < 0)
	{
	    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] Could not initialize pepper head shared memory" << RESETCOLOR  << std::endl;
	}
	else
	{
	    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] Pepper head shared memory initialized" << RESETCOLOR  << std::endl;
	    _pepperHeadPtr = mmap(0, SIZE, PROT_WRITE, MAP_SHARED, _shmfdPepperHead, 0);
	}

	_shmfdDepth2Laser = shm_open("Depth2LaserSharedMemory", O_RDWR, 0666);
	if(_shmfdDepth2Laser < 0)
	{
	    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] Could not initialize depth2Laser shared memory" << RESETCOLOR  << std::endl;
	}
	else
	{
	    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] depth to laser shared memory initialized" << RESETCOLOR  << std::endl;
	    _depth2LaserPtr = mmap(0, SIZE, PROT_WRITE, MAP_SHARED, _shmfdDepth2Laser, 0);
	}

	_shmfdPepperLocalizer = shm_open("PepperLocalizerSharedMemory", O_RDWR, 0666);
	if(_shmfdPepperLocalizer < 0)
	{
	    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] Could not initialize pepper localizer shared memory" << RESETCOLOR  << std::endl;
	}
	else
	{
	    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] pepper localizer shared memory initialized" << RESETCOLOR  << std::endl;
	    _pepperLocalizerPtr = mmap(0, SIZE, PROT_WRITE, MAP_SHARED, _shmfdPepperLocalizer, 0);
	}

	_shmfdPepperPlanner = shm_open("PepperPlannerSharedMemory", O_RDWR, 0666);
	if(_shmfdPepperPlanner < 0)
	{
	    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] Could not initialize pepper planer shared memory" << RESETCOLOR  << std::endl;
	}
	else
	{
	    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] pepper planner shared memory initialized" << RESETCOLOR  << std::endl;
	    _pepperPlannerPtr = mmap(0, SIZE, PROT_WRITE, MAP_SHARED, _shmfdPepperPlanner, 0);
	}
*/
    }

    void RobotToolkit::sendSharedMemory(std::string name, char value)
    {
	/*if(name == "PepperHeadSharedMemory")
	{
	    if(_shmfdPepperHead > 0)
		memcpy(_pepperHeadPtr, &value, sizeof(char));
	}

	else if(name == "Depth2LaserSharedMemory")
	{
	    if(_shmfdDepth2Laser > 0)
		memcpy(_depth2LaserPtr , &value, sizeof(char));
	}

	else if(name == "PepperLocalizerSharedMemory")
	{
	    if(_shmfdPepperLocalizer > 0)
		memcpy(_pepperLocalizerPtr , &value, sizeof(char));
	}

	else if(name == "PepperPlannerSharedMemory")
	{
	    if(_shmfdPepperPlanner > 0)
		memcpy(_pepperPlannerPtr, &value, sizeof(char));
	}*/
    }


    QI_REGISTER_OBJECT( RobotToolkit, _whoWillWin, setMasterURINet, startPublishing);
}
