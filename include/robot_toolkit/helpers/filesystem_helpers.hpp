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




#ifndef FILESYSTEM_HELPERS_HPP
#define FILESYSTEM_HELPERS_HPP

#include <qi/session.hpp>

#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/algorithm/string/replace.hpp>

#include "ros/ros.h"
#ifdef CATKIN_BUILD
#include <ros/package.h>
#endif

namespace Sinfonia
{
    namespace Helpers
    {
	namespace FileSystem
	{

	    static const long folderMaximumSize = 2000000000;

	    inline std::string& getURDF( std::string fileName )
	    {
		#ifdef CATKIN_BUILD
		    static std::string path = ros::package::getPath("robot_toolkit")+"/share/urdf/"+fileName;
		    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] " << "Found a catkin URDF " << path << std::endl;
		return path;
		#else
		    static std::string path = qi::path::findData( "/urdf/", fileName );
		    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] " << "Found a qibuild URDF " << path << std::endl;
		    return path;
		#endif
	    }
	    
	}
    } 
} 
#endif
