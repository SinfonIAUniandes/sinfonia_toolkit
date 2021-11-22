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

#define BOLDRED "\033[1m\033[31m"
#include "robot_toolkit/misc_tools/sonar/sonar_publisher.hpp"

namespace Sinfonia 
{
    namespace Publisher 
    {
	
	SonarPublisher::SonarPublisher(std::vector< std::string > topicNames)
	{
	    _isInitialized = false;
	    _topicNames = topicNames;
	}

	std::string SonarPublisher::getTopicName()
	{
	    std::string result;
	    result = "";
	    for(int i=0; i<_topicNames.size(); i++)
	    {
		result += _topicNames[i] + "//";
	    }
	    return result;
	}

	bool SonarPublisher::isInitialized() const
	{
	    return _isInitialized;
	}

	bool SonarPublisher::isSubscribed() const
	{
	    if (!_isInitialized) 
		return false;
	    bool result=false;
	    for(int i=0; i<_publishers.size(); i++)
	    {
		if(_publishers[i].getNumSubscribers() > 0)
		    result = true;
	    }
	    return result;
	}
	
	void SonarPublisher::publish(const std::vector< sensor_msgs::Range >& message)
	{
	    if ( _publishers.size() != message.size() )
	    {
		std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] Incorrect number of sonar range messages in sonar publisher. " << message.size() << "/" << _publishers.size() << std::endl;
		return;
	    }
	    for( size_t i=0; i<message.size(); ++i)
	    {
		_publishers[i].publish( message[i] );
	    }
	}
	
	void SonarPublisher::reset(ros::NodeHandle& nodeHandle)
	{
	    _publishers = std::vector<ros::Publisher>();
	    for( size_t i=0; i< _topicNames.size() ; ++i)
	    {
		_publishers.push_back( nodeHandle.advertise<sensor_msgs::Range>(_topicNames[i], 1) );
	    }

	    _isInitialized = true;
	}
	
	void SonarPublisher::shutdown()
	{
	    for(int i=0; i<_publishers.size(); i++)
	    {
		_publishers[i].shutdown();
	    }
	}
    }
}