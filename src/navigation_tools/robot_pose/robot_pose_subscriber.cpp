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

#include "robot_toolkit/navigation_tools/robot_pose/robot_pose_subscriber.hpp"

namespace Sinfonia 
{
    namespace Subscriber 
    {
	RobotPoseSubscriber::RobotPoseSubscriber(const std::string& name, const std::string& topic, const qi::SessionPtr& session): 
	  BaseSubscriber(name, topic, session)
	{
	    _topic = topic;
	    _pMemory = session->service("ALMemory");
	    _isInitialized = false;
	}

	void RobotPoseSubscriber::reset(ros::NodeHandle& nodeHandle)
	{
	    _subscriberRobotPose = nodeHandle.subscribe(_topic, 10, &RobotPoseSubscriber::robotPoseCallback, this);
	    _isInitialized = true;
	}

	void RobotPoseSubscriber::robotPoseCallback(const geometry_msgs::Pose2D message)
	{
	    std::vector<float> goal;
	    goal.push_back(message.x);
	    goal.push_back(message.y);
	    goal.push_back(message.theta);
	    qi::AnyValue value = qi::AnyValue::from(goal)	;
	    _pMemory.call<void>("raiseEvent", "NAOqiLocalizer/SetPose", value);
	    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] " << "Setting robot pose to to X -> " << goal[0] << " Y -> "  << goal[1] << " W-> " << goal[2] << std::endl;
	}
	
	void RobotPoseSubscriber::shutdown()
	{
	    _subscriberRobotPose.shutdown();
	    _isInitialized = false;
	}
    }
}