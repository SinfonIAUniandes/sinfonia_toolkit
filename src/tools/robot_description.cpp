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


#include "robot_toolkit/tools/robot_description.hpp"
#include "robot_toolkit/helpers/filesystem_helpers.hpp"


namespace Sinfonia
{

    namespace Tools
    {
	std::string getRobotDescription()
	{
	    std::string urdfPath;
	    static std::string robotDescription;
	    if(!robotDescription.empty())
		return robotDescription;
	    
	    urdfPath = Helpers::FileSystem::getURDF("pepper.urdf");

	    std::ifstream stream( (urdfPath).c_str() );
	    if (!stream)
	    {
		std::cerr << "failed to load robot description in joint_state_publisher: " << urdfPath << std::endl;
		return std::string();
	    }
	    robotDescription = std::string( (std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
	    return robotDescription;
	}

    }

}
