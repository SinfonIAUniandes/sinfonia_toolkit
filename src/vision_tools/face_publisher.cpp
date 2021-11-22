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

#include "robot_toolkit/vision_tools/face_publisher.hpp"

namespace Sinfonia
{
    namespace Publisher
    {
	
	FacePublisher::FacePublisher(std::string topicName)
	{
	    _topicName = topicName;
	}
	
	std::string FacePublisher::getTopicName()
	{
	    return _topicName;
	}
	
	bool FacePublisher::isInitialized() const
	{
	    return _isInitialized;
	}
	
	bool FacePublisher::isSubscribed() const
	{
	    if (!_isInitialized) 
		return false;
	    return _publisher.getNumSubscribers() > 0;
	}
	
	void FacePublisher::publish(robot_toolkit_msgs::face_detection_msgPtr message)
	{
	    _publisher.publish(*message);
	}
	
	void FacePublisher::reset(ros::NodeHandle& nodeHandle)
	{
	    _publisher = nodeHandle.advertise<robot_toolkit_msgs::face_detection_msg>( _topicName, 10 );
	    _isInitialized = true;
	}
	
	void FacePublisher::shutdown()
	{
	    _publisher.shutdown();
	}
    }
}