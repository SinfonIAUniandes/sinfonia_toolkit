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



#ifndef AUDIO_PUBLISHER_HPP
#define AUDIO_PUBLISHER_HPP

#include <string>

#include <naoqi_bridge_msgs/AudioBuffer.h>
#include <ros/ros.h>

namespace Sinfonia
{
    namespace Publisher
    {
	class MicPublisher
	{

	    public:
		MicPublisher();
		virtual ~MicPublisher() {}
		std::string topic();


		bool isInitialized() const;

		bool isSubscribed() const;

		void publish( naoqi_bridge_msgs::AudioBufferPtr message );

		void reset( ros::NodeHandle& nodeHandle );
		
		void shutdown();

	    protected:
		bool _isInitialized;
		ros::Publisher _publisher;
		std::string _topic;
	};

    } 
} 

#endif
