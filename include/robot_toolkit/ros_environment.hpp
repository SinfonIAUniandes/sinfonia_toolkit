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


#ifndef ROS_ENVIRONMENT_HPP
#define ROS_ENVIRONMENT_HPP

#define RESETCOLOR "\033[0m"
#define GREEN "\033[32m"
#define BOLDGREEN "\033[1m\033[32m"
#define HIGHGREEN "\033[92m"
#define BOLDRED "\033[1m\033[31m"
#define YELLOW "\033[33m"
#define BOLDYELLOW "\033[1m\033[33m"
#define BOLDCYAN "\033[1m\033[36m"
#define BOLDMAGENTA "\033[1m\033[35m"
#define BOLDBLUE "\033[1m\033[34m"

#include <ros/ros.h>

#include <qi/os.hpp>
#include <stdlib.h>
#include <boost/algorithm/string.hpp>

namespace Sinfonia
{
    namespace RosEnvironment
    {
	static std::string getROSIP(std::string networkInterface)
	{
	    if( networkInterface.empty() )
		networkInterface = "eth0";

	    typedef std::map< std::string, std::vector<std::string> > mapip;
	    
	    mapip mapIp = static_cast<mapip>(qi::os::hostIPAddrs());
	    
	    if( mapIp.find(networkInterface) == mapIp.end() )
	    {
		std::cerr << "Could not find network interface named " << networkInterface << ", possible interfaces are ... ";
		for( mapip::iterator i=mapIp.begin(); i!=mapIp.end(); ++i ) 
		    std::cerr << i->first <<  " ";
		std::cerr << std::endl;
		exit(1);
	    }

	    static const std::string ip = mapIp[networkInterface][0];
	    return ip;
	}

	static std::string _prefix = "";

	static void setPrefix( std::string prefix )
	{
	    _prefix = prefix;
	    std::cout << "set prefix successfully to " << _prefix << std::endl;
	}

	static std::string getPrefix()
	{
	    return _prefix;
	}

	static void setMasterURI( const std::string& uri, const std::string& networkInterface )
	{
	    if (ros::isInitialized() )
	    {
		std::cout << "stopping ros init" << std::endl;
		ros::shutdown();
	    }

	    setenv("ROS_MASTER_URI", uri.c_str(), 1);

	    std::string myMaster = "__master=" + uri;
	    std::map< std::string, std::string > reMap;
	    reMap["__master"] = uri;
	    reMap["__ip"] = ::Sinfonia::RosEnvironment::getROSIP(networkInterface);
	    
	    const char* namespaceEnvironment = std::getenv("ROS_NAMESPACE");
	    ros::init( reMap, (::Sinfonia::RosEnvironment::getPrefix()), ros::init_options::NoSigintHandler );
	    
	    ros::start();

	    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] " << "using master ip: " <<  ros::master::getURI() << std::endl;
	}

	static std::string getMasterURI( )
	{
	    return getenv("ROS_MASTER_URI");
	}

    }
}
#endif
