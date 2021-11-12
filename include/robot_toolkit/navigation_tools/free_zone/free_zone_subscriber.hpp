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

#ifndef FREE_ZONE_SUBSCRIBER_HPP
#define FREE_ZONE_SUBSCRIBER_HPP

#include "robot_toolkit/subscriber/base_subscriber.hpp"
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose2D.h>
#include "robot_toolkit_msgs/set_angles_msg.h"

namespace Sinfonia
{
    namespace Subscriber
    {
	class FreeZoneSubscriber: public BaseSubscriber<FreeZoneSubscriber>
	{
	    public:
		FreeZoneSubscriber( const std::string& name, const std::string& topic, const qi::SessionPtr& session );
		~FreeZoneSubscriber(){}

		void reset( ros::NodeHandle& nodeHandle );
		void freeZoneCallback( const geometry_msgs::Vector3 message );
		void shutdown();
		
		std::vector<float> getParameters(){
		    std::vector<float> result;
		    return result;
		}
		std::vector<float> setParameters(std::vector<float> parameters){
		    std::vector<float> result;
		    return result;
		}
		std::vector<float> setDefaultParameters(){
		    std::vector<float> result;
		    return result;
		}
		

	    private:
		
		geometry_msgs::Pose2D floatVector2Pose2D(std::vector<float> vector);
		geometry_msgs::Pose2D inversePose2D(geometry_msgs::Pose2D pose2D);
		geometry_msgs::Pose2D multiplyPose2d(geometry_msgs::Pose2D A, geometry_msgs::Pose2D B);
		std::string _topic;
		qi::AnyObject _pMotion;
		qi::AnyObject _pNavigation;
		ros::Subscriber _subscriberFreezone;
		ros::Publisher _publisher;
	}; 
    }
}
#endif
