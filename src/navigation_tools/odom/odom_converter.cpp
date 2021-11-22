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


#include "robot_toolkit/navigation_tools/odom/odom_converter.hpp"

#include <boost/foreach.hpp>
#define for_each BOOST_FOREACH

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace Sinfonia
{
    namespace Converter
    {	
	OdomConverter::OdomConverter(const std::string& name, const float& frequency, const qi::SessionPtr& session):
	BaseConverter( name, frequency, session )
	{
	    _pMotion =  session->service("ALMotion");
	}
	
	void OdomConverter::registerCallback(MessageAction::MessageAction action, callbackT callback)
	{
	    _callbacks[action] = callback;
	}
	
	void OdomConverter::callAll(const std::vector<MessageAction::MessageAction>& actions )
	{
	    double init = ros::Time::now().toSec();
	    callOdom();
	    for_each( MessageAction::MessageAction action, actions )
	    {
		_callbacks[action](_msgOdom);
	    }
	    //std::cout << BOLDYELLOW << "Topic: /odom " << BOLDCYAN << "elapsed time (s): "<< std::fixed << std::setprecision(8) << ros::Time::now().toSec() - init << std::endl;
	}
	
	void OdomConverter::callOdom()
	{
	    int frameWorld = 1;
	    bool useSensor = true;
	    std::vector<float> alOdometryData = _pMotion.call<std::vector<float> >( "getPosition", "Torso", frameWorld, useSensor );
	    
	    const ros::Time& odom_stamp = ros::Time::now();
	    std::vector<float> al_speed_data = _pMotion.call<std::vector<float> >( "getRobotVelocity" );
	    
	    const float& odomX  =  alOdometryData[0];
	    const float& odomY  =  alOdometryData[1];
	    const float& odomZ  =  alOdometryData[2];
	    const float& odomWX =  alOdometryData[3];
	    const float& odomWY =  alOdometryData[4];
	    const float& odomWZ =  alOdometryData[5];
	    
	    const float& dX = al_speed_data[0];
	    const float& dY = al_speed_data[1];
	    const float& dWZ = al_speed_data[2];

	    tf2::Quaternion tfQuaternion;
	    tfQuaternion.setRPY( odomWX, odomWY, odomWZ );
	    geometry_msgs::Quaternion odomQuaternion = tf2::toMsg( tfQuaternion );
	    
	    _msgOdom = boost::make_shared<nav_msgs::Odometry>();
	    _msgOdom->header.frame_id = "odom";
	    _msgOdom->child_frame_id = "base_link";
	    _msgOdom->header.stamp = odom_stamp;

	    _msgOdom->pose.pose.orientation = odomQuaternion;
	    _msgOdom->pose.pose.position.x = odomX;
	    _msgOdom->pose.pose.position.y = odomY;
	    _msgOdom->pose.pose.position.z = odomZ;
	    
	    _msgOdom->twist.twist.linear.x = dX;
	    _msgOdom->twist.twist.linear.y = dY;
	    _msgOdom->twist.twist.linear.z = 0;
	    
	    _msgOdom->twist.twist.angular.x = 0;
	    _msgOdom->twist.twist.angular.y = 0;
	    _msgOdom->twist.twist.angular.z = dWZ;
	}
	
	void OdomConverter::reset()
	{

	}


    }
    
}