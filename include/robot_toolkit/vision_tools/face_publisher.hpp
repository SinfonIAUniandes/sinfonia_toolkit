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



#ifndef FACE_PUBLISHER_HPP
#define FACE_PUBLISHER_HPP

#include <string>

#include  "robot_toolkit_msgs/face_detection_msg.h"
#include <ros/ros.h>

namespace Sinfonia
{
    namespace Publisher
    {

	class FacePublisher
	{

	    public:
		FacePublisher(std::string topicName);
		virtual ~FacePublisher() {}
		std::string getTopicName();


		bool isInitialized() const;

		bool isSubscribed() const;

		void publish( robot_toolkit_msgs::face_detection_msgPtr message );

		void reset( ros::NodeHandle& nodeHandle );
		
		void shutdown();

	    protected:
		bool _isInitialized;
		ros::Publisher _publisher;
		std::string _topicName;
	};
    } 
} 

#endif
