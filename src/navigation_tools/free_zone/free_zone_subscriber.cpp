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

#include "robot_toolkit/navigation_tools/free_zone/free_zone_subscriber.hpp"

namespace Sinfonia
{
    namespace Subscriber
    {
	FreeZoneSubscriber::FreeZoneSubscriber(const std::string& name, const std::string& topic, const qi::SessionPtr& session): 
	    BaseSubscriber(name, topic, session)
	{
	    _topic = topic;
	    _pMotion = session->service("ALMotion");
	    _pNavigation = session->service("ALNavigation");
	    _isInitialized = false;
	}

	void FreeZoneSubscriber::reset(ros::NodeHandle& nodeHandle)
	{
	    _subscriberFreezone = nodeHandle.subscribe(_topic, 10, &FreeZoneSubscriber::freeZoneCallback, this);
	    _publisher = nodeHandle.advertise<geometry_msgs::Pose2D>(_topic + "/result", 100);
	    _isInitialized = true;
	}
	
	void FreeZoneSubscriber::shutdown()
	{
	    _subscriberFreezone.shutdown();
	    _publisher.shutdown();
	    _isInitialized = false;
	}
	
	void FreeZoneSubscriber::freeZoneCallback(const geometry_msgs::Vector3 message)
	{
	    float desiredRadius = message.x;
	    float displacementCodenstraint = message.y;
	    
	    qi::AnyValue value = _pNavigation.call<qi::AnyValue>("getFreeZone", desiredRadius, displacementCodenstraint);
	    std::vector<float> floatworldToCenterFreeZone = value[2].toList<float>();
	    floatworldToCenterFreeZone.push_back(0.0);
	    geometry_msgs::Pose2D worldToCenterFreeZone = floatVector2Pose2D(floatworldToCenterFreeZone);
	    geometry_msgs::Pose2D worldToRobot = floatVector2Pose2D(_pMotion.call<qi::AnyValue>("getRobotPosition", true).toList<float>());
	    geometry_msgs::Pose2D worldToRobotInv = inversePose2D(worldToRobot);
	    geometry_msgs::Pose2D robotToFreeZoneCenter = multiplyPose2d(worldToRobotInv, worldToCenterFreeZone);
	    
	    _pMotion.async<void>("moveTo", robotToFreeZoneCenter.x, robotToFreeZoneCenter.y, 0.0);
	    _pMotion.call<void>("waitUntilMoveIsFinished");
	    robotToFreeZoneCenter.theta = 0;
	    _publisher.publish(robotToFreeZoneCenter);
	}
	
	geometry_msgs::Pose2D FreeZoneSubscriber::floatVector2Pose2D(std::vector<float> vector)
	{
	    geometry_msgs::Pose2D result;
	    result.x = vector[0];
	    result.y = vector[1];
	    result.theta = vector[2];
	    
	    return result;
	}
	
	geometry_msgs::Pose2D FreeZoneSubscriber::inversePose2D(geometry_msgs::Pose2D pose2D)
	{
	    geometry_msgs::Pose2D result;
	    
	    float theta = -pose2D.theta;
	    const float cos = std::cos(theta);
	    const float sin = std::sin(theta);
	    const float x   = pose2D.x;
	    const float y   = pose2D.y;
	   
	    result.x = -(x*cos - y*sin);
	    result.y = -(y*cos + x*sin);
	    result.theta = theta;
	    return result;
	}
	
	geometry_msgs::Pose2D FreeZoneSubscriber::multiplyPose2d(geometry_msgs::Pose2D A, geometry_msgs::Pose2D B)
	{
	    geometry_msgs::Pose2D result;
	    float cos = std::cos(A.theta);
	    float sin = std::sin(A.theta);
	    
	    result.x = A.x + cos*B.x - sin*B.y;
	    result.y = A.y + sin*B.x + cos*B.y;
	    result.theta = A.theta + B.theta;
	    
	    return result;
	}


    }
}