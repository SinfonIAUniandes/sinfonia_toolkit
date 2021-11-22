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


#ifndef PATH_PUBLISHER_HPP
#define PATH_PUBLISHER_HPP

#include <ros/ros.h>
#include "robot_toolkit_msgs/path_msg.h"

namespace Sinfonia
{
    namespace Publisher
    {

	class PathPublisher
	{

	    public:
		PathPublisher(std::string topicName);
		~PathPublisher(){}
		std::string getTopicName();

		bool isInitialized();
		
		virtual void publish( const robot_toolkit_msgs::path_msgPtr message);
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
