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


#ifndef TOOLS_HPP
#define TOOLS_HPP

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

#include <qi/anyobject.hpp>

namespace Sinfonia
{
    enum Topics 
    {
	Laser = 0,
	Camera,
	Sonar
    };

    namespace DataType 
    {
	enum DataType
	{
	    None = 0,
	    Float,
	    Int,
	    String,
	    Bool
	};
    }
} 

#if LIBQI_VERSION>24
    QI_TYPE_ENUM(Sinfonia::Topics);
    QI_TYPE_ENUM(Sinfonia::DataType::DataType);
#else
    QI_TYPE_ENUM_REGISTER(Sinfonia::Topics);
    QI_TYPE_ENUM_REGISTER(Sinfonia::DataType::DataType);
#endif

#endif
