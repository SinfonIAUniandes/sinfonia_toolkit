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

#ifndef SQUEDULED_CONVERTER_HPP
#define SQUEDULED_CONVERTER_HPP

#include "ros/ros.h"

namespace Sinfonia
{
    namespace Helpers
    {
	struct ScheduledConverter 
	    {
		ScheduledConverter(const ros::Time& schedule, size_t converterIndex):
		_schedule(schedule), _converterIndex(converterIndex)
		{
		    
		}

		bool operator < (const ScheduledConverter& spIn) const 
		{
		    return _schedule > spIn._schedule;
		}
		
		ros::Time _schedule;
		
		size_t _converterIndex;
	    };
    }
}


#endif