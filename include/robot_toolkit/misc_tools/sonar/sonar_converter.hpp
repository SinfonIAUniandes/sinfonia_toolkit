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


#ifndef SONAR_CONVERTER_HPP
#define SONAR_CONVERTER_HPP

#include <boost/foreach.hpp>

#include "robot_toolkit/converter/converter_base.hpp"
#include "robot_toolkit/message_actions.h"
#include <sensor_msgs/Range.h>

namespace Sinfonia
{
    namespace Converter
    {
	class SonarConverter : public BaseConverter<SonarConverter>
	{

	    typedef boost::function<void(std::vector<sensor_msgs::Range>&)> CallbackT;

	    public:
		SonarConverter( const std::string& name, const float& frequency, const qi::SessionPtr& session );
		~SonarConverter();

		void registerCallback( MessageAction::MessageAction action, CallbackT callback );

		void callAll( const std::vector<MessageAction::MessageAction>& actions );

		void reset( );
		
		void setConfig(std::vector<float> configs){}
		
		void shutdown(){}
		
		std::vector<float> setParameters(std::vector<float> parameters)
		{
		    std::vector<float> result;
		    return result;
		}
		std::vector<float> setAllParametersToDefault()
		{
		    std::vector<float> result;
		    return result;
		}
		std::vector<float> getParameters()
		{
		    std::vector<float> result;
		    return result;
		}	

	    private:
		bool _isSubscribed;
		
		qi::AnyObject _pMemory;
		qi::AnyObject _pSonar;
		
		std::vector<std::string> _keys;
		std::vector<std::string> _frames;
		std::vector<sensor_msgs::Range> _msgs;
		
		std::vector<float> fromAnyValueToFloatVector(qi::AnyValue& value, std::vector<float>& result);
		
		std::map<MessageAction::MessageAction, CallbackT> _callbacks;
		
		boost::shared_ptr<sensor_msgs::Range> _sonarMessage;
		
		void callSonar();
	};
    } 
} 

#endif
