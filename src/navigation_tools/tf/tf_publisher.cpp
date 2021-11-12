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


#include "robot_toolkit/navigation_tools/tf/tf_publisher.hpp"

namespace Sinfonia
{
    namespace Publisher
    {
	TfPublisher::TfPublisher(std::string topicName)
	{
	    _isInitialized = false;
	    _topicName = topicName;
	}
	
	TfPublisher::~TfPublisher()
	{

	}
	
	std::string TfPublisher::getTopicName()
	{
	    return _topicName;
	}
	
	bool TfPublisher::isInitialized()
	{
	    return _isInitialized;
	}

	void TfPublisher::publish(const std::vector< geometry_msgs::TransformStamped >& TfTransforms)
	{
	   
	    tf2_msgs::TFMessage message;
	    for (std::vector<geometry_msgs::TransformStamped>::const_iterator i = TfTransforms.begin(); i != TfTransforms.end(); ++i)
	    {
		message.transforms.push_back(*i);
	    }
	    _publisher.publish(message);
	}

	void TfPublisher::reset( ros::NodeHandle& nodeHandle )
	{
	    
	    _publisher = nodeHandle.advertise<tf2_msgs::TFMessage>(_topicName, 100);
	    _isInitialized = true;
	}

	bool TfPublisher::isSubscribed() const
	{
	    if (!_isInitialized) 
		return false;
	    return _publisher.getNumSubscribers() > 0;
	}
	
	void TfPublisher::shutdown()
	{
	    _publisher.shutdown();
	}


    }
} 
