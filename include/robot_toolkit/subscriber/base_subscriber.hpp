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


#ifndef BASE_SUBSCRIBER_HPP
#define BASE_SUBSCRIBER_HPP

#include <qi/session.hpp>

#include "robot_toolkit/tools/tools.hpp"

#include "robot_toolkit/helpers/toolkit_helpers.hpp"
namespace Sinfonia
{
    namespace Subscriber
    {
	template<class T>
	class BaseSubscriber
	{
	    public:
		BaseSubscriber( const std::string& name, const std::string& topic, qi::SessionPtr session )
		{
		    _name = name ;
		    _topic =  topic;
		    _isInitialized = false;
		    _session = session;
		}

		virtual ~BaseSubscriber() {}

		inline std::string name() const
		{
		    return _name;
		}

		inline std::string topic() const
		{
		    return _topic;
		}

		inline bool isInitialized() const
		{
		    return _isInitialized;
		}

	    protected:
		std::string _name, _topic;

		bool _isInitialized;

		qi::SessionPtr _session;
	};

    } 
}

#endif
