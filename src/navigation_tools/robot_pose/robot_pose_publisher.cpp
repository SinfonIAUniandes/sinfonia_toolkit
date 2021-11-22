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

#include "robot_toolkit/navigation_tools/robot_pose/robot_pose_publisher.hpp"

namespace Sinfonia
{
    namespace Publisher
    {
	RobotPosePublisher::RobotPosePublisher(std::string topicName)
	{
	    _isInitialized = false;
	    _topicName = topicName;
	}

	std::string RobotPosePublisher::getTopicName()
	{
	    return _topicName;
	}

	bool RobotPosePublisher::isInitialized()
	{
	    return _isInitialized;
	}

	void RobotPosePublisher::publish(const geometry_msgs::Vector3Ptr message)
	{
	    _publisher.publish(*message);
	}
	
	void RobotPosePublisher::reset(ros::NodeHandle& nodeHandle)
	{
	    _publisher = nodeHandle.advertise<geometry_msgs::Vector3>(_topicName, 100);
	    _isInitialized = true;
	}
	
	bool RobotPosePublisher::isSubscribed() const
	{
	    if (!_isInitialized) 
		return false;
	    return _publisher.getNumSubscribers() > 0;
	}
	
	void RobotPosePublisher::shutdown()
	{
	    _publisher.shutdown();
	}
    }
}
