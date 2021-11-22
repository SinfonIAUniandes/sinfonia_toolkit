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

#include "robot_toolkit/navigation_tools/odom/odom_publisher.hpp"

namespace Sinfonia 
{
    namespace Publisher 
    {
	OdomPublisher::OdomPublisher(std::string topicName)
	{
	    _topicName = topicName;
	}
	
	OdomPublisher::~OdomPublisher()
	{

	}
	
	std::string OdomPublisher::getTopicName()
	{
	    return _topicName;
	}
	
	bool OdomPublisher::isInitialized()
	{
	    return _isInitialized;
	}
	
	void OdomPublisher::publish(const nav_msgs::OdometryPtr odomMessage)
	{
	    _publisher.publish(*odomMessage);
	}
	
	void OdomPublisher::reset(ros::NodeHandle& nodeHandle)
	{
	    _publisher = nodeHandle.advertise<nav_msgs::Odometry>(_topicName, 10 );
	    _isInitialized = true;   
	}

	bool OdomPublisher::isSubscribed() const
	{
	    if (!_isInitialized) 
		return false;
	    return _publisher.getNumSubscribers() > 0;
	}
	
	void OdomPublisher::shutdown()
	{
	    _publisher.shutdown();
	}

    }
}