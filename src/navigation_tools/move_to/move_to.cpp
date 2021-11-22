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

#include "robot_toolkit/navigation_tools/move_to/move_to.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace Sinfonia
{
    namespace Subscriber
    {

	MoveToSubscriber::MoveToSubscriber( const std::string& name, const std::string& topic, const qi::SessionPtr& session, const boost::shared_ptr<tf2_ros::Buffer>& tf2Buffer):
	  BaseSubscriber( name, topic, session ),
	  _tf2Buffer(tf2Buffer)
	{
	    _pMotion = session->service("ALMotion");
	    //_tf2Buffer = tf2Buffer;
	}

	void MoveToSubscriber::reset( ros::NodeHandle& nodeHandle )
	{
	    _subscriberMoveTo = nodeHandle.subscribe( _topic, 10, &MoveToSubscriber::callback, this );
	    _isInitialized = true;
	}

	void MoveToSubscriber::callback( const geometry_msgs::PoseStampedConstPtr& poseMessage )
	{
	    if ( poseMessage->header.frame_id == "base_footprint" )
	    {
		double yaw = Helpers::Transform::getYaw(poseMessage->pose);

		std::cout << "going to move x: " <<  poseMessage->pose.position.x << " y: " << poseMessage->pose.position.y << " z: " << poseMessage->pose.position.z << " yaw: " << yaw << std::endl;
		_pMotion.async<void>("moveTo", poseMessage->pose.position.x, poseMessage->pose.position.y, yaw);
	    }
	    else
	    {
		geometry_msgs::PoseStamped poseMessageBf;
		bool canTransform = _tf2Buffer->canTransform("base_footprint", poseMessage->header.frame_id, ros::Time(0), ros::Duration(2) );
		if (!canTransform) 
		{
		    std::cout << "Cannot transform from " << poseMessage->header.frame_id << " to base_footprint" << std::endl;
		    return;
		}
		try
		{
		    _tf2Buffer->transform( *poseMessage, poseMessageBf, "base_footprint", ros::Time(0), poseMessage->header.frame_id );
		    double yaw = Helpers::Transform::getYaw(poseMessageBf.pose);
		    std::cout << BOLDCYAN << "[" << ros::Time::now().toSec() << "] " << "Odom to move x: " <<  poseMessageBf.pose.position.x << " y: " << poseMessageBf.pose.position.y << " z: " << poseMessageBf.pose.position.z << " yaw: " << yaw << std::endl;
		    _pMotion.async<void>("moveTo", poseMessageBf.pose.position.x, poseMessageBf.pose.position.y, yaw );
		} 
		catch( const tf2::LookupException& e)
		{
		    std::cout << e.what() << std::endl;
		    std::cout << "moveto position in frame_id " << poseMessage->header.frame_id << "is not supported in any other base frame than basefootprint" << std::endl;
		}
		catch( const tf2::ExtrapolationException& e)
		{
		    std::cout << "received an error on the time lookup" << std::endl;
		}
	    }
	}
	void MoveToSubscriber::shutdown()
	{
	    _subscriberMoveTo.shutdown();
	    _isInitialized = false;
	}
	

    } 
} 
