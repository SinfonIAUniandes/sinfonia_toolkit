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


#ifndef ODOM_PUBLISHER_HPP
#define ODOM_PUBLISHER_HPP


#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

namespace Sinfonia
{
    namespace Publisher
    {

	class OdomPublisher
	{

	    public:
		OdomPublisher(std::string topicName);
		~OdomPublisher();
		std::string getTopicName();

		bool isInitialized();
		
		virtual void publish(const nav_msgs::OdometryPtr odomMessage);
		virtual void reset( ros::NodeHandle& nodeHandle );
		virtual bool isSubscribed() const;
		virtual void shutdown();

	    private:
		ros::Publisher _publisher;

		std::string _topicName;

		bool _isInitialized;
	};

    }
}

#endif
