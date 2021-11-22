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


#ifndef TF_PUBLISHER_HPP
#define TF_PUBLISHER_HPP


#include <ros/ros.h>
#include <geometry_msgs/Transform.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf2_msgs/TFMessage.h>

namespace Sinfonia
{
    namespace Publisher
    {

	class TfPublisher
	{

	    public:
		TfPublisher(std::string topicName);
		~TfPublisher();
		std::string getTopicName();

		bool isInitialized();
		
		virtual void publish( const std::vector<geometry_msgs::TransformStamped>& TfTransforms);
		virtual void reset( ros::NodeHandle& nodeHandle );
		virtual void shutdown();
		virtual bool isSubscribed() const;

	    private:
		ros::Publisher _publisher;

		std::string _topicName;

		bool _isInitialized;
		
	};

    }
}

#endif
