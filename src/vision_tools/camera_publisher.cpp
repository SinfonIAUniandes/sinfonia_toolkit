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

#include "robot_toolkit/vision_tools/camera_publisher.hpp"
namespace enc = sensor_msgs::image_encodings;
namespace Sinfonia
{
    namespace Publisher
    {
	
	CameraPublisher::CameraPublisher(std::string topicName)
	{
	    _topicName = topicName;
	}
	
	std::string CameraPublisher::getTopicName()
	{
	    return _topicName;
	}
	
	bool CameraPublisher::isInitialized()
	{
	    return _isInitialized;
	}
	
	void CameraPublisher::publish(const sensor_msgs::ImagePtr img, const sensor_msgs::CameraInfoPtr cameraInfo)
	{
	    _publisher.publish(*img, *cameraInfo);
	    
	}
	
	void CameraPublisher::reset(ros::NodeHandle& nodeHandle)
	{
	    image_transport::ImageTransport it( nodeHandle );
	    _publisher = it.advertiseCamera( _topicName, 1 );
	    _isInitialized = true;
	}

	bool CameraPublisher::isSubscribed() const
	{
	    if (!_isInitialized) 
		return false;
	    return _publisher.getNumSubscribers() > 0;
	}
	
	void CameraPublisher::shutdown()
	{
	    _publisher.shutdown();
	}
	
	void CameraPublisher::setCameraSource(int cameraSource)
	{
	    _cameraSource = cameraSource;
	}
    }
}