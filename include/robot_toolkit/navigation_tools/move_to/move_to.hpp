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

#ifndef MOVETO_SUBSCRIBER_HPP
#define MOVETO_SUBSCRIBER_HPP


#include "robot_toolkit/subscriber/base_subscriber.hpp"
#include "robot_toolkit/helpers/transform_helpers.hpp"


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/buffer.h>

namespace Sinfonia
{
    namespace Subscriber
    {

	class MoveToSubscriber: public BaseSubscriber<MoveToSubscriber>
	{
	    public:
		MoveToSubscriber( const std::string& name, const std::string& topic, const qi::SessionPtr& session, const boost::shared_ptr<tf2_ros::Buffer>& tf2Buffer );
		~MoveToSubscriber(){}

		void reset( ros::NodeHandle& nodeHandle );
		void callback( const geometry_msgs::PoseStampedConstPtr& poseMessage );
		void shutdown();
		std::vector<float> setParameters(std::vector<float> parameters)
		{
		    std::vector<float> result;
		    return result;
		}
		std::vector<float> setDefaultParameters()
		{
		    std::vector<float> result;
		    return result;
		}
		std::vector<float> getParameters()
		{
		    std::vector<float> result;
		    return result;
		}

	    private:
		qi::AnyObject _pMotion;
		ros::Subscriber _subscriberMoveTo;
		boost::shared_ptr<tf2_ros::Buffer> _tf2Buffer;
	}; 
    }
}
#endif
