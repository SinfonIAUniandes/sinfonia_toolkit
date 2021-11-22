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


#ifndef TF_CONVERTER_HPP
#define TF_CONVERTER_HPP

#include "robot_toolkit/converter/converter_base.hpp"
#include "robot_toolkit/tools/robot_description.hpp"
#include "robot_toolkit/message_actions.h"


#include <urdf/model.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/buffer.h>
#include <robot_state_publisher/robot_state_publisher.h>

namespace Sinfonia
{
    namespace Converter
    {

	class TfConverter : public BaseConverter<TfConverter>
	{

	typedef boost::function<void(std::vector<geometry_msgs::TransformStamped>&) > callbackT;

	typedef boost::shared_ptr<tf2_ros::Buffer> bufferPtr;

	typedef std::map<std::string, std::shared_ptr<urdf::JointMimic> > mimicMap;

	public:
	    TfConverter( const std::string& name, const float& frequency, const bufferPtr& tf2Buffer, const qi::SessionPtr& session );
	    ~TfConverter();

	    virtual void reset( );
	    void registerCallback( const MessageAction::MessageAction action, callbackT callBack );
	    void callAll( const std::vector<MessageAction::MessageAction>& actions );
	    
	    void setConfig(std::vector<float> configs){}
		void setPublishOdom();
	    
	    std::vector<float> setParameters(std::vector<float> parameters){}
	    std::vector<float> setAllParametersToDefault(){}
	    std::vector<float> getParameters(){}
	    
	    void shutdown(){}

	private:    
	    void setTransforms(const std::map<std::string, double>& jointPositions, const ros::Time& time, const std::string& tfPrefix);
	    void setFixedTransforms(const std::string& tfPrefix, const ros::Time& time);
	    void addChildren(const KDL::SegmentMap::const_iterator segment);
	    void callTF();
	    
	    bufferPtr _tf2Buffer;
		bool _publishOdom;
  
	    qi::AnyObject _pMotion;

	    std::map<MessageAction::MessageAction, callbackT> _callbacks;
	    std::map<std::string, robot_state_publisher::SegmentPair> _segments, _segmentsFixed;
	    
	    std::string _robotDescription;
	    
	    mimicMap _mimic;
	    
	    sensor_msgs::JointState _msgJointStates;
	    
	    std::vector<geometry_msgs::TransformStamped> _tfTransforms;
	    
		std::vector<std::string> _tfFrames;
		
	};


    } 
} 

#endif
