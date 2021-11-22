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


#ifndef FACE_TRACKER_HPP
#define FACE_TRACKER_HPP


#include <qi/session.hpp>

namespace Sinfonia
{
    class BasicAwareness
    {
	public:
	    BasicAwareness(const qi::SessionPtr& session);
	    void start();
	    void stop();
	    
	private: 
	    qi::SessionPtr _session;
	    
	    qi::AnyObject _pAwareness;
	    
	    std::string _targetName;
	    std::string _mode;
	    std::vector<int> _params;
    };
}

#endif