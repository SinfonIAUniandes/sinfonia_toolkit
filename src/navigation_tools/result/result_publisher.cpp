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

#include "robot_toolkit/navigation_tools/result/result_publisher.hpp"

namespace Sinfonia
{
    namespace Publisher
    {
	ResultPublisher::ResultPublisher(std::string topicName)
	{
	    _isInitialized = false;
	    _topicName = topicName;
	}
	std::string ResultPublisher::getTopicName()
	{
	    return _topicName;
	}

	bool ResultPublisher::isInitialized() const
	{
	    return _isInitialized;
	}

	bool ResultPublisher::isSubscribed() const
	{
	    if (!_isInitialized) 
		return false;
	    return _publisher.getNumSubscribers() > 0;
	}
	void ResultPublisher::publish(std_msgs::StringPtr message)
	{
	    _publisher.publish( *message );
	}
	void ResultPublisher::reset(ros::NodeHandle& nodeHandle)
	{
	    _publisher = nodeHandle.advertise<std_msgs::String>( _topicName, 10 );
	    _isInitialized = true;
	}
	void ResultPublisher::shutdown()
	{
	    _publisher.shutdown();
	}
    }
    
}