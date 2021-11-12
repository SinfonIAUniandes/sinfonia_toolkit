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




#include <qi/applicationsession.hpp>
#include <qi/anymodule.hpp>
#include "robot_toolkit/tools/tools.hpp"

#include "robot_toolkit/robot_toolkit.hpp"


#include <boost/program_options.hpp>
#include "robot_toolkit/naoqi_environment.hpp"


int main(int argc, char **argv)
{
    std::cout << std::fixed << std::setprecision(3);
    qi::ApplicationSession app(argc, argv);
    
    std::vector<std::string> argsOut;
    ros::removeROSArgs( argc, argv, argsOut );
    
    namespace programOptions = boost::program_options;
    programOptions::options_description optionsDescription("Options");
    optionsDescription.add_options()
	("help,h", "print help message")
	("roscore_ip,r", programOptions::value<std::string>(), "set the ip of the roscore to use")
	("network_interface,i", programOptions::value<std::string>()->default_value("eth0"),  "set the network interface over which to connect")
	("namespace,n", programOptions::value<std::string>()->default_value("robot_toolkit_node"), "set an explicit namespace in case ROS namespace variables cannot be used")
    ("publish_odom,o", programOptions::value<bool>(), "publish wheels odometry as main odom");

    programOptions::variables_map variablesMap;
    try
    {
	programOptions::store( programOptions::parse_command_line(argc, argv, optionsDescription), variablesMap );
    }
    catch (boost::program_options::invalid_command_line_syntax& e)
    {
	std::cout << "Error 0x01: Invialid Sintax" << e.what() << std::endl;
	throw ros::Exception(e.what());
    }
    catch (boost::program_options::unknown_option& e)
    {
	std::cout << "Error 0x02: Unknown option" << e.what() << std::endl;
	throw ros::Exception(e.what());
    }

    if( variablesMap.count("help") )
    {
	std::cout << "This is the help message for the SinfonIA RobotToolkit" << std::endl << optionsDescription << std::endl;
	exit(0);
    }
    #if LIBQI_VERSION>24
	app.startSession();
    #else
	app.start();
    #endif
    boost::shared_ptr<Sinfonia::RobotToolkit> robotToolkit = boost::make_shared<Sinfonia::RobotToolkit>(app.session(), variablesMap["namespace"].as<std::string>());
    app.session()->registerService("robot_toolkit", robotToolkit);
    
    if ( variablesMap.count("roscore_ip") )
    {
	std::string roscoreIp = variablesMap["roscore_ip"].as<std::string>();
	std::string networkInterface = variablesMap["network_interface"].as<std::string>();

	std::cout << BOLDYELLOW << "using ip address: " << BOLDCYAN << roscoreIp << " @ " << networkInterface << RESETCOLOR << std::endl;
	
    if(variablesMap["publish_odom"].as<bool>())
        robotToolkit->setPublishOdom();        
	robotToolkit->init();
	robotToolkit->setMasterURINet( "http://"+roscoreIp+":11311", networkInterface); 
	robotToolkit->startInitialTopics();
    }
    else
    {
	std::cout << BOLDRED << "No ip address given. Run qicli call to set the master uri" << RESETCOLOR << std::endl;
	robotToolkit->init();
    }
    app.run();
    robotToolkit->stopService();
    app.session()->close();
    return 0;
}

