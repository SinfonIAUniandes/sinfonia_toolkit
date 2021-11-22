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

#include "robot_toolkit/audio_tools/mic/mic_localization_publisher.hpp"

namespace Sinfonia 
{
    namespace Publisher 
    {
	
	MicLocalizationPublisher::MicLocalizationPublisher(std::string topic)
	{
	    _topic = topic;
	    _isInitialized = false;
	}

	std::string MicLocalizationPublisher::topic()
	{
	    return _topic; 
	}
	
	bool MicLocalizationPublisher::isInitialized() const
	{
	    return _isInitialized;
	}
	
	bool MicLocalizationPublisher::isSubscribed() const
	{
	    if (!_isInitialized) 
		return false;
	    return _publisher.getNumSubscribers() > 0;
	}
	
	void MicLocalizationPublisher::publish( robot_toolkit_msgs::audio_localization_msgPtr message )
	{
	    _publisher.publish( *message );
	}
	
	void MicLocalizationPublisher::reset(ros::NodeHandle& nodeHandle)
	{
	    _publisher = nodeHandle.advertise< robot_toolkit_msgs::audio_localization_msg>( _topic, 10 );
	    _isInitialized = true;
	}
	
	void MicLocalizationPublisher::shutdown()
	{
	    _publisher.shutdown();
	}
    }
}