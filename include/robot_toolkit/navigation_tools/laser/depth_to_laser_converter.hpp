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

#ifndef DEPTH_TO_LASER_CONVERTER_HPP
#define DEPTH_TO_LASER_CONVERTER_HPP

#include <boost/foreach.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>
#include <ros/console.h>

#include "robot_toolkit/converter/converter_base.hpp"
#include "robot_toolkit/message_actions.h"

#include <ros/package.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <boost/assign/list_of.hpp>

#include <image_transport/image_transport.h>
#include "../../helpers/vision_helpers.hpp"

#include <qi/anyvalue.hpp>

#include "robot_toolkit_msgs/camera_parameters_msg.h"

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>

namespace Sinfonia
{
    namespace Converter
    {
	template<typename T> struct DepthTraits {};
	
	template<>
	struct DepthTraits<uint16_t>
	{
	    static inline bool isValid(uint16_t depth) { return depth != 0; }
	    static inline float toMeters(uint16_t depth) { return depth * 0.001f; }
	    static inline uint16_t fromMeters(float depth) { return (depth * 1000.0f) + 0.5f; }
	    static inline void initializeBuffer(std::vector<uint8_t>& buffer) {}
	};

	template<>
	struct DepthTraits<float>
	{
	    static inline bool isValid(float depth) { return std::isfinite(depth); }
	    static inline float toMeters(float depth) { return depth; }
	    static inline float fromMeters(float depth) { return depth; }

	    static inline void initializeBuffer(std::vector<uint8_t>& buffer)
	    {
		float* start = reinterpret_cast<float*>(&buffer[0]);
		float* end = reinterpret_cast<float*>(&buffer[0] + buffer.size());
		std::fill(start, end, std::numeric_limits<float>::quiet_NaN());
	    }
	};
	
	class DepthToLaserConverter : public BaseConverter<DepthToLaserConverter>
	{
	    typedef boost::function<void(sensor_msgs::LaserScanPtr)> CallbackT;

	    public:
		DepthToLaserConverter( const std::string& name, const float& frequency, const qi::SessionPtr& session, int resolution);
		~DepthToLaserConverter();

		void registerCallback( MessageAction::MessageAction action, CallbackT callback );

		void callAll( const std::vector<MessageAction::MessageAction>& actions );

		void reset( );
		
		void setConfig(std::vector<float> configs);
		
		void shutdown(){}
		
		std::vector<float> setParameters(std::vector<float> parameters);
		std::vector<float> setAllParametersToDefault();
		std::vector<float> getParameters();
		
	    private:
		std::map<MessageAction::MessageAction, CallbackT> _callbacks;

		qi::AnyObject _pVideo;
		int _cameraSource;
		int _resolution;
		int _colorSpace;
		std::string _handle;

		std::string _msgColorspace;
		int _cvMatType;
		
		sensor_msgs::CameraInfoPtr _cameraInfo;
		boost::shared_ptr<sensor_msgs::LaserScan> _laserMessage;
		
		void callCamera();
		sensor_msgs::CameraInfoPtr loadCameraInfo();
		const sensor_msgs::CameraInfoPtr& getEmptyInfo();
		
		Helpers::VisionHelpers::NaoqiImage fromAnyValueToNaoqiImage(qi::AnyValue& value);
		
		
		sensor_msgs::CameraInfoPtr createCameraInfoDEPTHVGA();
		sensor_msgs::CameraInfoPtr createCameraInfoDEPTHQVGA();
		sensor_msgs::CameraInfoPtr createCameraInfoDEPTHQQVGA();
		
		
		image_geometry::PinholeCameraModel _cameraModel; ///< image_geometry helper class for managing sensor_msgs/CameraInfo messages.

		float _scanTime; ///< Stores the time between scans.
		float _rangeMin; ///< Stores the current minimum range to use.
		float _rangeMax; ///< Stores the current maximum range to use.
		int _scanHeight; ///< Number of pixel rows to use when producing a laserscan from an area.
		std::string _outputFrameId; ///< Output frame_id for each laserscan.  This is likely NOT the camera's frame_id.template<typename T> struct DepthTraits {};
		
		sensor_msgs::LaserScanPtr convertMessage(const sensor_msgs::ImageConstPtr depthCameraMessage, const sensor_msgs::CameraInfoConstPtr cameraInfoMessage);
		void setScanTime(const float scanTime);
		void setRangeLimits(const float rangeMin, const float rangeMax);
		void setScanHeight(const int scanHeight);
		void setOutputFrame(const std::string outputFrameId);
		double magnitudeOfRay(const cv::Point3d& ray) const;
		double angleBetweenRays(const cv::Point3d& ray1, const cv::Point3d& ray2) const;
		bool usePoint(const float newValue, const float oldValue, const float rangeMin, const float rangeMax) const;
		
		/**
		* Converts the depth image to a laserscan using the DepthTraits to assist.
		* 
		* This uses a method to inverse project each pixel into a LaserScan angular increment.  This method first projects the pixel
		* forward into Cartesian coordinates, then calculates the range and angle for this point.  When multiple points coorespond to
		* a specific angular measurement, then the shortest range is used.
		* 
		* @param depth_msg The UInt16 or Float32 encoded depth message.
		* @param cam_model The image_geometry camera model for this image.
		* @param scan_msg The output LaserScan.
		* @param scan_height The number of vertical pixels to feed into each angular_measurement.
		* 
		*/
		template<typename T>
		void convert(const sensor_msgs::ImageConstPtr depthCameraMessage, const image_geometry::PinholeCameraModel& cameraModel, const sensor_msgs::LaserScanPtr& laserScanMessage, const int& scanHeight) const
		{
		    // Use correct principal point from calibration
		    float centerX = cameraModel.cx();
		    float centerY = cameraModel.cy();

		    // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
		    double unitScaling = DepthTraits<T>::toMeters( T(1) );
		    float constantX = unitScaling / cameraModel.fx();
		    float constantY = unitScaling / cameraModel.fy();

		    const T* depthRow = reinterpret_cast<const T*>(&depthCameraMessage->data[0]);
		    int rowStep = depthCameraMessage->step / sizeof(T);

		    int offset = (int)(cameraModel.cy()-scanHeight/2);
		    depthRow += offset*rowStep; // Offset to center of image

		    for(int v = offset; v < offset+_scanHeight; v++, depthRow += rowStep)
		    {
			for (int u = 0; u < (int)depthCameraMessage->width; u++) // Loop over each pixel in row
			{	
			    T depth = depthRow[u];
			    
			    double r = depth; // Assign to pass through NaNs and Infs
			    double th = -atan2((double)(u - centerX) * constantX, unitScaling); // Atan2(x, z), but depth divides out
			    int index = (th - laserScanMessage->angle_min) / laserScanMessage->angle_increment;
			    
			    if (DepthTraits<T>::isValid(depth))// Not NaN or Inf
			    { 
				// Calculate in XYZ
				double x = (u - centerX) * depth * constantX;
				double z = DepthTraits<T>::toMeters(depth);
				
				// Calculate actual distance
				r = sqrt(pow(x, 2.0) + pow(z, 2.0));
			    }
		    
			    // Determine if this point should be used.
			    if(usePoint(r, laserScanMessage->ranges[index], laserScanMessage->range_min, laserScanMessage->range_max))
			    {
				laserScanMessage->ranges[index] = r;
			    }
			}
		    }
		};
	};
    } 
} 

#endif
