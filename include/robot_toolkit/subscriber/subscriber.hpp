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


#ifndef SUBSCRIBER_HPP
#define SUBSCRIBER_HPP

#include <string>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

namespace Sinfonia
{
    namespace Subscriber
    {

	class Subscriber
	{

	    public:
		template<typename T>
		Subscriber( T subscriber )
		{
		    _subscriberPtr = boost::make_shared<SubscriberModel<T> >(subscriber); 
		}

		bool isInitialized() const
		{
		    return _subscriberPtr->isInitialized();
		}


		void reset( ros::NodeHandle& nodeHandle )
		{
		    std::cout << BOLDYELLOW << "[" << ros::Time::now().toSec() << "] " << name() << " is resetting" << std::endl;
		    _subscriberPtr->reset( nodeHandle );
		    std::cout << BOLDYELLOW << "[" << ros::Time::now().toSec() << "] " << name() << " reset" << std::endl;
		}


		std::string name() const
		{
		    return _subscriberPtr->name();
		}


		std::string topic() const
		{
		    return _subscriberPtr->topic();
		}
		
		void shutdown()
		{
		    _subscriberPtr->shutdown();
		}
		
		std::vector<float> getParameters()
		{
		    return _subscriberPtr->getParameters();
		}
		
		std::vector<float> setParameters(std::vector<float> parameters)
		{
		    return _subscriberPtr->setParameters(parameters);
		}
		
		std::vector<float> setDefaultParameters()
		{
		    return _subscriberPtr->setDefaultParameters();
		}

		friend bool operator==( const Subscriber& lhs, const Subscriber& rhs )
		{
		    if ( lhs.name() == rhs.name() || lhs.topic() == rhs.topic() )
			return true;
		    return false;
		}

	    private:
		struct SubscriberConcept
		{
		    virtual ~SubscriberConcept(){}
		    virtual bool isInitialized() const = 0;
		    virtual void reset( ros::NodeHandle& nh ) = 0;
		    virtual std::string name() const = 0;
		    virtual std::string topic() const = 0;
		    virtual void shutdown() = 0;
		    virtual std::vector<float> getParameters() = 0;
		    virtual std::vector<float> setParameters(std::vector<float> parameters) = 0;
		    virtual std::vector<float> setDefaultParameters() = 0;
		};

		template<typename T>
		struct SubscriberModel : public SubscriberConcept
		{
		    SubscriberModel( const T& other )
		    {
			_subscriber = other;
		    }

		    std::string name() const
		    {
			return _subscriber->name();
		    }

		    std::string topic() const
		    {
			return _subscriber->topic();
		    }

		    bool isInitialized() const
		    {
			return _subscriber->isInitialized();
		    }

		    void reset( ros::NodeHandle& nodeHandle )
		    {
			_subscriber->reset( nodeHandle );
		    }
		    
		    void shutdown()
		    {
			_subscriber->shutdown();
		    }
		    
		    std::vector<float> getParameters()
		    {
			return _subscriber->getParameters();
		    }
		    
		    std::vector<float> setParameters(std::vector<float> parameters)
		    {
			return _subscriber->setParameters(parameters);
		    }
		    
		    std::vector<float> setDefaultParameters()
		    {
			return _subscriber->setDefaultParameters();
		    }
		    
		    T _subscriber;
		};

		boost::shared_ptr<SubscriberConcept> _subscriberPtr;

	}; 

    } 
} 

#endif
