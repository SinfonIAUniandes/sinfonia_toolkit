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

#include "robot_toolkit/navigation_tools/laser/laser_merged_converter.hpp"

#define for_each BOOST_FOREACH

namespace Sinfonia
{
    namespace Converter
    {
	static const char* _laserMemoryKeys[] = {
					    // RIGHT LASER
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg01/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg01/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg02/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg02/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg03/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg03/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg04/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg04/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg05/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg05/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg06/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg06/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg07/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg07/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg08/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg08/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg09/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg09/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg10/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg10/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg11/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg11/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg12/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg12/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg13/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg13/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg14/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg14/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg15/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg15/Y/Sensor/Value",
					    // FRONT LASER
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg01/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg01/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg02/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg02/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg03/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg03/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg04/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg04/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg05/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg05/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg06/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg06/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg07/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg07/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg08/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg08/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg09/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg09/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg10/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg10/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg11/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg11/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg12/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg12/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg13/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg13/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg14/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg14/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg15/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg15/Y/Sensor/Value",
					    // LEFT LASER
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg01/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg01/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg02/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg02/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg03/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg03/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg04/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg04/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg05/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg05/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg06/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg06/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg07/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg07/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg08/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg08/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg09/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg09/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg10/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg10/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg11/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg11/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg12/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg12/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg13/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg13/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg14/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg14/Y/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg15/X/Sensor/Value",
					    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg15/Y/Sensor/Value",
							};
							
	LaserMergedConverter::LaserMergedConverter(const std::string& name, const float& frequency, const qi::SessionPtr& session):
	    BaseConverter( name, frequency, session )
	{
	    _pMemory = _session->service("ALMemory");
	    _laserMessage = boost::make_shared<sensor_msgs::LaserScan>();	    
	}
	
	void LaserMergedConverter::registerCallback(MessageAction::MessageAction action, LaserMergedConverter::CallbackT callback)
	{
	    _callbacks[action] = callback;
	}

	void LaserMergedConverter::callAll(const std::vector<MessageAction::MessageAction>& actions)
	{
	    double init = ros::Time::now().toSec();
	    callLaser();
	    for_each(MessageAction::MessageAction action, actions)
	    {
		_callbacks[action](_laserMessage);		
	    }
	    //std::cout << BOLDYELLOW << "Topic: /laser " << BOLDCYAN << "elapsed time (s): "<< std::fixed << std::setprecision(8) << ros::Time::now().toSec() - init << std::endl;
	}
	
	

	void LaserMergedConverter::callLaser()
	{
	    
	    
	    float alphaMin = -2.0944;
	    float alphaMax = 2.0944;
	    int rangesLaserSize = 61;
	    float alpha = (2*alphaMax) / rangesLaserSize;
	    float rangeLaserMin = 0.1;
	    float rangeLaserMax = 8.0;
	    
	    std::vector<float> rangesLaser;
	    rangesLaser.resize((int)rangesLaserSize);
	    
	    static const std::vector<std::string> laserKeysValue(_laserMemoryKeys, _laserMemoryKeys+90);
	    std::vector<float> resultValue;
	    try
	    {
		qi::AnyValue anyvalues = _pMemory.call<qi::AnyValue>("getListData", laserKeysValue);
		fromAnyValueToFloatVector(anyvalues, resultValue);
	    }
	    catch (const std::exception& e) 
	    {
		std::cerr << "Exception caught in LaserConverter: " << e.what() << std::endl;
		return;
	    }
	    
	    
	    /*_laserMessage->header.stamp = ros::Time::now();
	    _laserMessage->header.frame_id = "base_footprint";
	    _laserMessage->angle_min = -2.0944;
	    _laserMessage->angle_max = 2.0944;   
	    _laserMessage->angle_increment = (2*2.0944) / (15+15+15+8+8); 
	    _laserMessage->range_min = 0.1;
	    _laserMessage->range_max = 8.0;
	    _laserMessage->ranges.resize(61);
	    _laserMessage->ranges = std::vector<float>(61, -1.0f);*/
	    size_t pos = 0;

	    for( size_t i=0; i<30; i=i+2, ++pos)
	    {
		const float lx = resultValue[28-i]; 
		const float ly = resultValue[28-i+1];
		float bx = lx*std::cos(-1.757) - ly*std::sin(-1.757) - 0.018;
		float by = lx*std::sin(-1.757) + ly*std::cos(-1.757) - 0.090;
		float dist = std::sqrt( std::pow(bx,2) + std::pow(by,2) );
		rangesLaser[pos] = dist;
	    }

	    pos = pos+8;

	    for( size_t i=0; i<30; i=i+2, ++pos)
	    {
		const float lx = resultValue[58-i];
		const float ly = resultValue[58-i+1];
		float bx = lx + 0.056 ;
		float by = ly;
		float dist = std::sqrt( std::pow(bx,2) + std::pow(by,2) );
		rangesLaser[pos] = dist;
	    }

	    pos = pos+8;

	    for( size_t i=0; i<30; i=i+2, ++pos)
	    {
		const float lx = resultValue[88-i];
		const float ly = resultValue[88-i+1];
		float bx = lx*std::cos(1.757) - ly*std::sin(1.757) - 0.018;
		float by = lx*std::sin(1.757) + ly*std::cos(1.757) + 0.090;
		float dist = std::sqrt( std::pow(bx,2) + std::pow(by,2) );
		rangesLaser[pos] = dist;
	    }
	    
	    
	    try
	    {
		std::vector<float> rangesDepth;
		qi::AnyValue rangesReadings = _pMemory.call<qi::AnyValue>("getData", "NAOqiDepth2Laser/Ranges");
		qi::AnyValue minAngleReading = _pMemory.call<qi::AnyValue>("getData", "NAOqiDepth2Laser/MinAngle");
		qi::AnyValue maxAngleReading = _pMemory.call<qi::AnyValue>("getData", "NAOqiDepth2Laser/MaxAngle");
		qi::AnyValue numRangesReading = _pMemory.call<qi::AnyValue>("getData", "NAOqiDepth2Laser/NumRanges");
		qi::AnyValue maxRangeReading = _pMemory.call<qi::AnyValue>("getData", "NAOqiDepth2Laser/MaxRange");
		
		rangesDepth = rangesReadings.toList<float>();
		
		float betaMin = minAngleReading.toFloat();
		float betaMax = maxAngleReading.toFloat();
		float rangesDepthSize = numRangesReading.toFloat();
		float beta = (betaMax - betaMin)/rangesDepthSize;
		float rangeLaserMin = 0.1;
		float rangeLaserMax = maxRangeReading.toFloat(); 
		
		int N = 512;
		float psi = (2*alphaMax)/((float)N);
		
		_laserMessage = boost::make_shared<sensor_msgs::LaserScan>();
		_laserMessage->header.stamp = ros::Time::now();
		_laserMessage->header.frame_id = "base_footprint";
		_laserMessage->angle_min = alphaMin;
		_laserMessage->angle_max = alphaMax;
		_laserMessage->angle_increment = psi;
		_laserMessage->range_min = 0.1;
		_laserMessage->range_max = rangeLaserMax;
		_laserMessage->ranges.resize(N);
		_laserMessage->scan_time = 0.01;
		_laserMessage->time_increment =  0.01/rangesDepthSize;
		
		
		
		int A = (int)((betaMin - alphaMin)/psi);
		float betaP = betaMax/(((float)N/2)- (float)A);
		
		for(int i=0; i<N; i++)
		{
		    _laserMessage->ranges[i] = 80.0f;
		}
		
		for(int i=0; i<(int)rangesDepthSize; i++)
		{
		    int iP = (int)round((((float)i*(beta))/betaP)) + A;
		    if(iP < N)
			_laserMessage->ranges[iP] = rangesDepth[i];
		}
		
		int depthLowerLimit = A;
		int depthHighLimit = (int)round((((float)(rangesDepthSize - 1)*(beta))/betaP)) + A;
		int leftLaserLimit = (int)((float)(14)*(float(N-1.0)/float(rangesLaserSize)));
		int rigthLaserLimit = (int)((float)(47)*(float(N-1.0)/float(rangesLaserSize)));
		
		std::vector<float> leftLaserIndexs;
		std::vector<float> rightLaserIdexs;
		
		for(int i = 0; i<rangesLaser.size(); i++)
		{
		    if(rangesLaser[i] > 1.0) 
		    {
			rangesLaser[i] = 80.0;
		    }
		}
		
		for(int i=0; i<rangesLaserSize; i++)
		{
		    int ip = (int)((float)(i)*(float(N-1.0)/float(rangesLaserSize)));
		    if((i > 38) || (i < 23))
		    {
			_laserMessage->ranges[ip] = rangesLaser[i];
			if(i<=14)
			    leftLaserIndexs.push_back(ip);
			else
			    rightLaserIdexs.push_back(ip);
		    }
		}
		
		
		
		for(int i=leftLaserLimit+1; i<depthLowerLimit; i++)
		{
		    _laserMessage->ranges[i] = -1.0f;
		}
		for(int i=depthHighLimit+1; i<rigthLaserLimit; i++)
		{
		    _laserMessage->ranges[i] = -1.0f;
		}
		
		// Interpolation :)
		for(int i=0; i<leftLaserIndexs.size()-1; i++)
		{
		    _laserMessage->ranges[(int)((leftLaserIndexs[i] + leftLaserIndexs[i+1])/2.0)] = (_laserMessage->ranges[leftLaserIndexs[i]] + _laserMessage->ranges[leftLaserIndexs[i+1]])/2.0;
		}
		for(int i=0; i<rightLaserIdexs.size()-1; i++)
		{
		    _laserMessage->ranges[(int)((rightLaserIdexs[i] + rightLaserIdexs[i+1])/2.0)] = (_laserMessage->ranges[rightLaserIdexs[i]] + _laserMessage->ranges[rightLaserIdexs[i+1]])/2.0;
		}
	    }
	    catch (qi::FutureUserException)
	    {
		std::cout << BOLDYELLOW << "[" << ros::Time::now().toSec() << "] NAOqiDepth2Laser not available" << std::endl;
	    }
	}
	
	
	void LaserMergedConverter::reset()
	{
	    _laserMessage->header.frame_id = "base_footprint";
	    _laserMessage->angle_min = -2.0944;
	    _laserMessage->angle_max = 2.0944;   
	    _laserMessage->angle_increment = (2*2.0944) / (15+15+15+8+8); 
	    _laserMessage->range_min = 0.1;
	    _laserMessage->range_max = 1.5;
	    _laserMessage->ranges = std::vector<float>(61, -1.0f);
	}

	std::vector< float > LaserMergedConverter::fromAnyValueToFloatVector(qi::AnyValue& value, std::vector< float >& result)
	{
	    qi::AnyReferenceVector anyrefs = value.asListValuePtr();
	    for(int i=0; i<anyrefs.size();i++)
	    {
		try
		{
		    result.push_back(anyrefs[i].content().toFloat());
		}
		catch(std::runtime_error& e)
		{
		    result.push_back(-1.0);
		    std::cout << e.what() << "=> set to -1" << std::endl;
		}
	    }
	    return result;
	}
	
    }
}
