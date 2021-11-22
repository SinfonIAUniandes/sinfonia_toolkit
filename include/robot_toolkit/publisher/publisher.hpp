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


#ifndef PUBLISHER_HPP
#define PUBLISHER_HPP

#include <string>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

#include "robot_toolkit/tools/tools.hpp"

namespace Sinfonia
{
    namespace Publisher
    {
	class Publisher
	{

	    public:

		template<typename T>
		Publisher( const T& publisher )
		{
		    _publisherPtr = boost::make_shared<PublisherModel<T> >(publisher);
		}
		
		bool isInitialized() const
		{
		    return _publisherPtr->isInitialized();
		}

		bool isSubscribed() const
		{
		    return _publisherPtr->isSubscribed();
		}

		void reset( ros::NodeHandle& nodeHandle )
		{
		    std::cout << BOLDYELLOW << "[" << ros::Time::now().toSec() << "] " << getTopicName() << " is resetting" << RESETCOLOR <<std::endl;
		    _publisherPtr->reset( nodeHandle );
		    std::cout << BOLDYELLOW << "[" << ros::Time::now().toSec() << "] " << getTopicName() << " reset" << RESETCOLOR << std::endl;
		}

		std::string getTopicName() const
		{
		    return _publisherPtr->getTopicName();
		}
		
		void shutdown() const
		{
		    _publisherPtr->shutdown();
		}
		friend bool operator==( const Publisher& lhs, const Publisher& rhs )
		{
		    if (lhs.getTopicName() == rhs.getTopicName())
			return true;
		    return false;
		}

		friend bool operator==( const boost::shared_ptr<Publisher>& lhs, const boost::shared_ptr<Publisher>& rhs )
		{
		    return operator==( *lhs, *rhs );
		}

	    private:

		struct PublisherConcept
		{
		    virtual ~PublisherConcept(){}
		    virtual bool isInitialized() const = 0;
		    virtual bool isSubscribed() const = 0;
		    virtual void reset( ros::NodeHandle& nh ) = 0;
		    virtual std::string getTopicName() const = 0;
		    virtual void shutdown() = 0;
		};

		template<typename T>
		struct PublisherModel : public PublisherConcept
		{
		    PublisherModel( const T& other )
		    {
			_publisher = other;
		    }

		    std::string getTopicName() const
		    {
			return _publisher->getTopicName();
		    }

		    bool isInitialized() const
		    {
			return _publisher->isInitialized();
		    }

		    bool isSubscribed() const
		    {
			return _publisher->isSubscribed();
		    }

		    void reset( ros::NodeHandle& nodeHandler )
		    {
			_publisher->reset(nodeHandler);
		    }
		    
		    void shutdown()
		    {
			_publisher->shutdown();
		    }

		    T _publisher;
		};

		boost::shared_ptr<PublisherConcept> _publisherPtr;

	};
    } 
} 

#endif
