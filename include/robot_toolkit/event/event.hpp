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

#ifndef EVENT_HPP
#define EVENT_HPP

#include <string>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include "robot_toolkit/message_actions.h"

namespace Sinfonia
{
    namespace Event
    {

	class Event
	{

	    public:

		template<typename T>
		Event( T event ):
		    _eventPtr( boost::make_shared<EventModel<T> >(event) )
		{}

		void resetPublisher( ros::NodeHandle& nh )
		{
		    _eventPtr->resetPublisher(nh);
		}
		
		void shutdownPublisher()
		{
		    _eventPtr->shutdownPublisher();
		}
		
		void shutdownEvents()
		{
		    _eventPtr->shutdownEvents();
		}

		void startProcess()
		{
		    _eventPtr->startProcess();
		}
		
		bool isStarted()
		{
		    return _eventPtr->isStarted();
		}
		
		void setDefaultParameters()
		{
		    _eventPtr->setDefaultParameters();
		}
		
		void setParameters(std::vector<int> parameters)
		{
		    _eventPtr->setParameters(parameters);
		}
		
		void stopProcess( )
		{
		    _eventPtr->stopProcess();
		}

		void isPublishing(bool state)
		{
		    _eventPtr->isPublishing(state);
		}


	    private:

		struct EventConcept
		{
		    virtual ~EventConcept(){}
		    virtual void resetPublisher(ros::NodeHandle& nh) = 0;
		    virtual void shutdownPublisher() = 0;
		    virtual void startProcess() = 0;
		    virtual void stopProcess() = 0;
		    virtual bool isStarted() = 0;
		    virtual void setDefaultParameters() = 0;
		    virtual void setParameters(std::vector<int> parameters) = 0;
		    virtual void isPublishing(bool state) = 0;
		    virtual void shutdownEvents() = 0;
		    
		};

		template<typename T>
		struct EventModel : public EventConcept
		{
		    EventModel( const T& other ):
		      _converter( other )
		    {}
		    
		    
		    
		    void resetPublisher( ros::NodeHandle& nh )
		    {
			_converter->resetPublisher(nh);
		    }
		    
		    void shutdownPublisher( )
		    {
			_converter->shutdownPublisher();
		    }

		    void startProcess( )
		    {
			_converter->startProcess();
		    }

		    void stopProcess( )
		    {
			_converter->stopProcess();
		    }
		    
		    bool isStarted()
		    {
			return _converter->isStarted();
		    }
		    
		    void setDefaultParameters()
		    {
			_converter->setDefaultParameters();
		    }
		    
		    void shutdownEvents()
		    {
			_converter->shutdownEvents();
		    }
		    
		    void setParameters(std::vector<int> parameters)
		    {
			_converter->setParameters(parameters);
		    }
		    
		    void isPublishing(bool state)
		    {
			_converter->isPublishing(state);
		    }

		    T _converter;
		};

		boost::shared_ptr<EventConcept> _eventPtr;

	};
    }
} 

#endif
