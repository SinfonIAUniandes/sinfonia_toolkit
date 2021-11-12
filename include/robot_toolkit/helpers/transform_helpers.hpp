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



#ifndef TRANSFORM_HELPERS_HPP
#define TRANSFORM_HELPERS_HPP

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace Sinfonia
{
    namespace Helpers
    {
	namespace Transform
	{

	    inline double getYaw(const geometry_msgs::Pose& pose)
	    {
		double yaw, _pitch, _roll;
		tf2::Matrix3x3(tf2::Quaternion(pose.orientation.x, pose.orientation.y,
						pose.orientation.z, pose.orientation.w)).getEulerYPR(yaw, _pitch, _roll);
		return yaw;
	    }

	    inline double getYaw( const geometry_msgs::Transform& pose)
	    {
		double yaw, _pitch, _roll;
		tf2::Matrix3x3(tf2::Quaternion(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w)).getEulerYPR(yaw, _pitch, _roll);
		return yaw;
	    }

	}
    }
} 

#endif
