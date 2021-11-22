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

#include "robot_toolkit/navigation_tools/laser/depth_to_laser_converter.hpp"
#include <boost/foreach.hpp>
#define for_each BOOST_FOREACH

namespace Sinfonia
{
    namespace Converter
    {
	DepthToLaserConverter::DepthToLaserConverter(const std::string& name, const float& frequency, const qi::SessionPtr& session, int resolution): 
	  BaseConverter(name, frequency, session)
	{	    
	    _pVideo = session->service("ALVideoDevice");
	    _laserMessage = boost::make_shared<sensor_msgs::LaserScan>();
	    setFrequency(frequency);
	    _colorSpace = Helpers::VisionHelpers::kDepthColorSpace;
	    _msgColorspace = "16UC1";
	    _cvMatType = CV_16U;
	    _resolution = resolution;
	    _cameraSource = Helpers::VisionHelpers::kDepthCamera;
	    _cameraInfo = loadCameraInfo();
	    setAllParametersToDefault(); 
	    setOutputFrame("CameraDepth_frame");
	   
	}
	
	DepthToLaserConverter::~DepthToLaserConverter()
	{
	    if (!_handle.empty())
	    {
		_pVideo.call<qi::AnyValue>("unsubscribe", _handle);
		std::cout << BOLDYELLOW << "[" << ros::Time::now().toSec() << "] " << "Unsubscribe camera handle: " << _handle << std::endl;
		_handle.clear();
	    }
	}

	void DepthToLaserConverter::registerCallback(MessageAction::MessageAction action, Converter::DepthToLaserConverter::CallbackT callback)
	{
	    _callbacks[action] = callback;
	}
	void DepthToLaserConverter::callAll(const std::vector< MessageAction::MessageAction >& actions)
	{
	    if (_handle.empty() )
	    {
		std::cerr << _name << "Camera Handle is empty - cannot retrieve image" << std::endl;
		std::cerr << _name << "Might be a NAOqi problem. Try to restart the ALVideoDevice." << std::endl;
		return;
	    }
	    callCamera();
	    for_each(MessageAction::MessageAction action, actions)
	    {
		_callbacks[action](_laserMessage);
	    }
	}
	void DepthToLaserConverter::reset()
	{
	    if (!_handle.empty())
	    {
		_pVideo.call<qi::AnyValue>("unsubscribe", _handle);
		_handle.clear();
	    }
	    _handle = _pVideo.call<std::string>("subscribeCamera", _name, _cameraSource, _resolution, _colorSpace, (int)_frequency);
	}
	
	void DepthToLaserConverter::setConfig(std::vector< float > configs)
	{
	    _resolution = (int)configs[0];
	    _cameraInfo = loadCameraInfo();
	}
	
	std::vector< float > DepthToLaserConverter::setParameters(std::vector<float> parameters)
	{
	    setScanTime(parameters[0]); 
	    setRangeLimits(parameters[1], parameters[2]);
	    setScanHeight(parameters[3]);
	    
	    return getParameters();	    
	}

	std::vector< float > DepthToLaserConverter::setAllParametersToDefault()
	{
	    setScanTime(1.0);
	    setRangeLimits(0.45, 10.0);
	    setScanHeight(120);
	    
	    return getParameters();
	}

	std::vector< float > DepthToLaserConverter::getParameters()
	{
	    std::vector<float> result;
	    result.push_back(_scanTime);
	    result.push_back(_rangeMin);
	    result.push_back(_rangeMax);
	    result.push_back(_scanHeight);
	    return result;
	}

	void DepthToLaserConverter::callCamera()
	{
	    qi::AnyValue imageAnyValue = _pVideo.call<qi::AnyValue>("getImageRemote", _handle);
	    Helpers::VisionHelpers::NaoqiImage image;
	    try
	    {
		image = fromAnyValueToNaoqiImage(imageAnyValue);
	    }
	    catch(std::runtime_error& e)
	    {
		std::cout << "Cannot retrieve image" << std::endl;
		return;
	    }

	    cv::Mat cvImage(image.height, image.width, _cvMatType, image.buffer);
	    
	    sensor_msgs::ImagePtr imageMsg;
	    imageMsg = cv_bridge::CvImage(std_msgs::Header(), _msgColorspace, cvImage).toImageMsg();
	    imageMsg->header.frame_id = "CameraDepth_frame";

	    imageMsg->header.stamp = ros::Time::now();
	    _cameraInfo->header.stamp = imageMsg->header.stamp;    
	    _laserMessage = convertMessage(imageMsg, _cameraInfo);
	}
	
	const sensor_msgs::CameraInfoPtr& DepthToLaserConverter::getEmptyInfo()
	{
	    static const boost::shared_ptr<sensor_msgs::CameraInfo> camInfoMsg = boost::make_shared<sensor_msgs::CameraInfo>();
	    return camInfoMsg;
	}
	
	Helpers::VisionHelpers::NaoqiImage DepthToLaserConverter::fromAnyValueToNaoqiImage(qi::AnyValue& value)
	{
	    qi::AnyReferenceVector anyReference;
	    Helpers::VisionHelpers::NaoqiImage result;
	    std::ostringstream stringStream;
	    try
	    {
		anyReference = value.asListValuePtr();
	    }
	    catch(std::runtime_error& e)
	    {
		stringStream << "Could not transform AnyValue into list: " << e.what();
		throw std::runtime_error(stringStream.str());
	    }
	    
	    qi::AnyReference ref;

	    ref = anyReference[0].content();
	    if(ref.kind() == qi::TypeKind_Int)
	    {
		result.width = ref.asInt32();
	    }
	    else
	    {
		stringStream << "Could not retrieve width";
		throw std::runtime_error(stringStream.str());
	    }

	    ref = anyReference[1].content();
	    if(ref.kind() == qi::TypeKind_Int)
	    {
		result.height = ref.asInt32();
	    }
	    else
	    {
		stringStream << "Could not retrieve height";
		throw std::runtime_error(stringStream.str());
	    }

	    ref = anyReference[2].content();
	    if(ref.kind() == qi::TypeKind_Int)
	    {
		result.numberOfLayers = ref.asInt32();
	    }
	    else
	    {
		stringStream << "Could not retrieve number of layers";
		throw std::runtime_error(stringStream.str());
	    }

	    ref = anyReference[3].content();
	    if(ref.kind() == qi::TypeKind_Int)
	    {
		result.colorSpace = ref.asInt32();
	    }
	    else
	    {
		stringStream << "Could not retrieve colorspace";
		throw std::runtime_error(stringStream.str());
	    }

	    ref = anyReference[4].content();
	    if(ref.kind() == qi::TypeKind_Int)
	    {
		result.timeStampS = ref.asInt32();
	    }
	    else
	    {
		stringStream << "Could not retrieve timestamp_s";
		throw std::runtime_error(stringStream.str());
	    }

	    ref = anyReference[5].content();
	    if(ref.kind() == qi::TypeKind_Int)
	    {
		result.timeStampUs = ref.asInt32();
	    }
	    else
	    {
		stringStream << "Could not retrieve timestamp_us";
		throw std::runtime_error(stringStream.str());
	    }

	    ref = anyReference[6].content();
	    if(ref.kind() == qi::TypeKind_Raw)
	    {
		result.buffer = (void*)ref.asRaw().first;
	    }
	    else
	    {
		stringStream << "Could not retrieve buffer";
		throw std::runtime_error(stringStream.str());
	    }

	    ref = anyReference[7].content();
	    if(ref.kind() == qi::TypeKind_Int)
	    {
		result.camId = ref.asInt32();
	    }
	    else
	    {
		stringStream << "Could not retrieve camId";
		throw std::runtime_error(stringStream.str());
	    }

	    ref = anyReference[8].content();
	    if(ref.kind() == qi::TypeKind_Float)
	    {
		result.fovLeft = ref.asFloat();
	    }
	    else
	    {
		stringStream << "Could not retrieve fov_left";
		throw std::runtime_error(stringStream.str());
	    }

	    ref = anyReference[9].content();
	    if(ref.kind() == qi::TypeKind_Float)
	    {
		result.fovTop = ref.asFloat();
	    }
	    else
	    {
		stringStream << "Could not retrieve fov_top";
		throw std::runtime_error(stringStream.str());
	    }

	    ref = anyReference[10].content();
	    if(ref.kind() == qi::TypeKind_Float)
	    {
		result.fovRight = ref.asFloat();
	    }
	    else
	    {
		stringStream << "Could not retrieve fov_right";
		throw std::runtime_error(stringStream.str());
	    }

	    ref = anyReference[11].content();
	    if(ref.kind() == qi::TypeKind_Float)
	    {
		result.fovBottom = ref.asFloat();
	    }
	    else
	    {
		stringStream << "Could not retrieve fov_bottom";
		throw std::runtime_error(stringStream.str());
	    }
	    return result;
	}
	
	sensor_msgs::CameraInfoPtr DepthToLaserConverter::loadCameraInfo()
	{
	    std::string resolutionName;
	    if(_resolution == Helpers::VisionHelpers::kQVGA)
	    {
		resolutionName = "kQVGA";
	    }
	    else if(_resolution == Helpers::VisionHelpers::kQQVGA)
	    {
		resolutionName = "kQQVGA";
	    }
	    else if(_resolution == Helpers::VisionHelpers::kQQQVGA)
	    {
		resolutionName = "kQQQVGA";
	    }
	    else if(_resolution == Helpers::VisionHelpers::kQQQQVGA)
	    {
		resolutionName = "kQQQQVGA";
	    }
	    else
	    {
		std::cout << BOLDYELLOW << "[" << ros::Time::now().toSec() << "] " << "Resolution not supported " << _cameraSource << " and res: " << _resolution << std::endl;
		return getEmptyInfo();
	    }
	    
	    boost::property_tree::ptree config;
	    std::string path = ros::package::getPath("robot_toolkit")+"/share/camera_info/depth_camera_info.json";
	    try
	    {
		boost::property_tree::read_json(path, config);
	    }
	    catch(std::exception& e)
	    {
		std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << "Not Found  camera info json " << path << std::endl;
		return getEmptyInfo();
	    }
	    
	    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] " << "Found a camera info JSON " << path << std::endl;
	    
	    static const boost::shared_ptr<sensor_msgs::CameraInfo> cameraInfoMessage = boost::make_shared<sensor_msgs::CameraInfo>();
	    
	    cameraInfoMessage->header.frame_id = "Camera_depth_optical_frame";
	    
	    cameraInfoMessage->width = config.get<int>(resolutionName + ".width", 0);
	    cameraInfoMessage->height = config.get<int>(resolutionName + ".height", 0);
	    std::string distortion = config.get<std::string>(resolutionName + ".distortion_model", "None");
	    if(distortion.compare("plumb_bob"))
	    {
		cameraInfoMessage->distortion_model = "plumb_bob";
	    }
	    else if(distortion.compare("rational_polynomial"))
	    {
		cameraInfoMessage->distortion_model = "rational_polynomial"; 
	    }
	    else
	    {
		std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << "Distortion model not supported " << path << std::endl;
	    }
	    
	    cameraInfoMessage->D = boost::assign::list_of(config.get<double>(resolutionName + ".D.0", 0))(config.get<double>(resolutionName + ".D.1", 0))(config.get<double>(resolutionName + ".D.2", 0))(config.get<double>(resolutionName + ".D.3", 0))(config.get<double>(resolutionName + ".D.4", 0)).convert_to_container<std::vector<double> >();
	    
	    cameraInfoMessage->K[0] = config.get<double>(resolutionName + ".K.0", 0);
	    cameraInfoMessage->K[1] = config.get<double>(resolutionName + ".K.1", 0);
	    cameraInfoMessage->K[2] = config.get<double>(resolutionName + ".K.2", 0);
	    cameraInfoMessage->K[3] = config.get<double>(resolutionName + ".K.3", 0);
	    cameraInfoMessage->K[4] = config.get<double>(resolutionName + ".K.4", 0);
	    cameraInfoMessage->K[5] = config.get<double>(resolutionName + ".K.5", 0);
	    cameraInfoMessage->K[6] = config.get<double>(resolutionName + ".K.6", 0);
	    cameraInfoMessage->K[7] = config.get<double>(resolutionName + ".K.7", 0);
	    cameraInfoMessage->K[8] = config.get<double>(resolutionName + ".K.8", 0);
	    
	    cameraInfoMessage->R[0] = config.get<double>(resolutionName + ".R.0", 0);
	    cameraInfoMessage->R[1] = config.get<double>(resolutionName + ".R.1", 0);
	    cameraInfoMessage->R[2] = config.get<double>(resolutionName + ".R.2", 0);
	    cameraInfoMessage->R[3] = config.get<double>(resolutionName + ".R.3", 0);
	    cameraInfoMessage->R[4] = config.get<double>(resolutionName + ".R.4", 0);
	    cameraInfoMessage->R[5] = config.get<double>(resolutionName + ".R.5", 0);
	    cameraInfoMessage->R[6] = config.get<double>(resolutionName + ".R.6", 0);
	    cameraInfoMessage->R[7] = config.get<double>(resolutionName + ".R.7", 0);
	    cameraInfoMessage->R[8] = config.get<double>(resolutionName + ".R.8", 0);
	    
	    cameraInfoMessage->P[0] = config.get<double>(resolutionName + ".P.0", 0);
	    cameraInfoMessage->P[1] = config.get<double>(resolutionName + ".P.1", 0);
	    cameraInfoMessage->P[2] = config.get<double>(resolutionName + ".P.2", 0);
	    cameraInfoMessage->P[3] = config.get<double>(resolutionName + ".P.3", 0);
	    cameraInfoMessage->P[4] = config.get<double>(resolutionName + ".P.4", 0);
	    cameraInfoMessage->P[5] = config.get<double>(resolutionName + ".P.5", 0);
	    cameraInfoMessage->P[6] = config.get<double>(resolutionName + ".P.6", 0);
	    cameraInfoMessage->P[7] = config.get<double>(resolutionName + ".P.7", 0);
	    cameraInfoMessage->P[8] = config.get<double>(resolutionName + ".P.8", 0);
	    cameraInfoMessage->P[9] = config.get<double>(resolutionName + ".P.9", 0);
	    cameraInfoMessage->P[10] = config.get<double>(resolutionName + ".P.10", 0);
	    cameraInfoMessage->P[11] = config.get<double>(resolutionName + ".P.11", 0);
	    
	    cameraInfoMessage->binning_x = config.get<int>(resolutionName + ".binning_x", 0);
	    cameraInfoMessage->binning_y = config.get<int>(resolutionName + ".binning_y", 0);
	    
	    cameraInfoMessage->roi.x_offset = config.get<int>(resolutionName + ".roi.x_offset", 0);
	    cameraInfoMessage->roi.y_offset = config.get<int>(resolutionName + ".roi.y_offset", 0);
	    cameraInfoMessage->roi.height = config.get<int>(resolutionName + ".roi.height", 0);
	    cameraInfoMessage->roi.width = config.get<int>(resolutionName + ".roi.width", 0);
	    cameraInfoMessage->roi.do_rectify = config.get<bool>(resolutionName + ".roi.do_rectify", 0);
	    
	    return cameraInfoMessage;
	}
	
	/**
	* Converts the information in a depth image (sensor_msgs::Image) to a sensor_msgs::LaserScan.
	* 
	* This function converts the information in the depth encoded image (UInt16 or Float32 encoding) into
	* a sensor_msgs::LaserScan as accurately as possible.  To do this, it requires the synchornized Image/CameraInfo
	* pair associated with the image.
	* 
	* @param depthCameraMessage UInt16 or Float32 encoded depth image.
	* @param cameraInfoMessage CameraInfo associated with depth_msg
	* @return sensor_msgs::LaserScanPtr for the center row(s) of the depth image.
	* 
	*/
	sensor_msgs::LaserScanPtr DepthToLaserConverter::convertMessage(const sensor_msgs::ImageConstPtr depthCameraMessage, const sensor_msgs::CameraInfoConstPtr cameraInfoMessage)
	{
	    // Set camera model
	    _cameraModel.fromCameraInfo(cameraInfoMessage);
	    // Calculate angle_min and angle_max by measuring angles between the left ray, right ray, and optical center ray
	    cv::Point2d rawPixelLeft(0, _cameraModel.cy());
	    cv::Point2d rectPixelLeft = _cameraModel.rectifyPoint(rawPixelLeft);
	    
	    cv::Point3d leftRay = _cameraModel.projectPixelTo3dRay(rectPixelLeft);
	    
	    cv::Point2d rawPixelRight(depthCameraMessage->width-1, _cameraModel.cy());
	    cv::Point2d rectPixelRight = _cameraModel.rectifyPoint(rawPixelRight);
	    cv::Point3d rightRay = _cameraModel.projectPixelTo3dRay(rectPixelRight);
	    
	    cv::Point2d rawPixelCenter(_cameraModel.cx(), _cameraModel.cy());
	    cv::Point2d rectPixelCenter = _cameraModel.rectifyPoint(rawPixelCenter);
	    cv::Point3d centerRay = _cameraModel.projectPixelTo3dRay(rectPixelCenter);
	    
	    double angleMax = angleBetweenRays(leftRay, centerRay);
	    double angleMin = -angleBetweenRays(centerRay, rightRay); // Negative because the laserscan message expects an opposite rotation of that from the depth image
	    
	    // Fill in laserscan message
	    sensor_msgs::LaserScanPtr scanMessage(new sensor_msgs::LaserScan());
	    scanMessage->header = depthCameraMessage->header;
	    if(_outputFrameId.length() > 0)
	    {
		scanMessage->header.frame_id = _outputFrameId;
	    }
	    scanMessage->angle_min = angleMin;
	    scanMessage->angle_max = angleMax;
	    scanMessage->angle_increment = (scanMessage->angle_max - scanMessage->angle_min) / (depthCameraMessage->width - 1);
	    scanMessage->time_increment = 0.0;
	    scanMessage->scan_time = _scanTime;
	    scanMessage->range_min = _rangeMin;
	    scanMessage->range_max = _rangeMax;
	    
	    // Check scan_height vs image_height
	    if(_scanHeight/2 > _cameraModel.cy() || _scanHeight/2 > depthCameraMessage->height - _cameraModel.cy())
	    {
		std::stringstream ss;
		ss << "scan_height ( " << _scanHeight << " pixels) is too large for the image height.";
		throw std::runtime_error(ss.str());
	    }

	    // Calculate and fill the ranges
	    uint32_t rangesSize = depthCameraMessage->width;
	    scanMessage->ranges.assign(rangesSize, std::numeric_limits<float>::quiet_NaN());
	    
	    if (depthCameraMessage->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
	    {
		convert<uint16_t>(depthCameraMessage, _cameraModel, scanMessage, _scanHeight);
	    }
	    else if (depthCameraMessage->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
	    {
		convert<float>(depthCameraMessage, _cameraModel, scanMessage, _scanHeight);
	    }
	    else
	    {
		std::stringstream ss;
		ss << "Depth image has unsupported encoding: " << depthCameraMessage->encoding;
		throw std::runtime_error(ss.str());
	    }
	    return scanMessage;
	}
	
	/**
	* Computes euclidean length of a cv::Point3d (as a ray from origin)
	* 
	* This function computes the length of a cv::Point3d assumed to be a vector starting at the origin (0,0,0).
	* 
	* @param ray The ray for which the magnitude is desired.
	* @return Returns the magnitude of the ray.
	* 
	*/
	double DepthToLaserConverter::magnitudeOfRay(const cv::Point3d& ray) const
	{
	    return sqrt(pow(ray.x, 2.0) + pow(ray.y, 2.0) + pow(ray.z, 2.0));
	}

	/**
	* Computes the angle between two cv::Point3d
	* 
	* Computes the angle of two cv::Point3d assumed to be vectors starting at the origin (0,0,0).
	* Uses the following equation: angle = arccos(a*b/(|a||b|)) where a = ray1 and b = ray2.
	* 
	* @param ray1 The first ray
	* @param ray2 The second ray
	* @return The angle between the two rays (in radians)
	* 
	*/
	double DepthToLaserConverter::angleBetweenRays(const cv::Point3d& ray1, const cv::Point3d& ray2) const
	{
	    double dotProduct = ray1.x*ray2.x + ray1.y*ray2.y + ray1.z*ray2.z;
	    double magnitude1 = magnitudeOfRay(ray1);
	    double magnitude2 = magnitudeOfRay(ray2);;
	    return acos(dotProduct / (magnitude1 * magnitude2));
	}

	
	/**
	* Determines whether or not new_value should replace old_value in the LaserScan.
	* 
	* Uses the values of range_min, and range_max to determine if new_value is a valid point.  Then it determines if
	* new_value is 'more ideal' (currently shorter range) than old_value.
	* 
	* @param newValue The current calculated range.
	* @param oldValue The current range in the output LaserScan.
	* @param rangeMin The minimum acceptable range for the output LaserScan.
	* @param rangeMax The maximum acceptable range for the output LaserScan.
	* @return If true, insert new_value into the output LaserScan.
	* 
	*/
	bool DepthToLaserConverter::usePoint(const float newValue, const float oldValue, const float rangeMin, const float rangeMax) const
	{  
	    // Check for NaNs and Infs, a real number within our limits is more desirable than these.
	    bool newFinite = std::isfinite(newValue);
	    bool oldFinite = std::isfinite(oldValue);
	
	    // Infs are preferable over NaNs (more information)
	    if(!newFinite && !oldFinite) // Both are not NaN or Inf.
	    { 
		if(!std::isnan(newValue)) // new is not NaN, so use it's +-Inf value.
		{ 
		    return true;
		}
		return false; // Do not replace old_value
	    }
	
	    // If not in range, don't bother
	    bool rangeCheck = rangeMin <= newValue && newValue <= rangeMax;
	    if(!rangeCheck)
	    {
		return false;
	    }

	    if(!oldFinite) // New value is in range and finite, use it.
	    { 
		return true;
	    }

	    // Finally, if they are both numerical and new_value is closer than old_value, use new_value.
	    bool shorterCheck = newValue < oldValue;
	    return shorterCheck;
	}

	/**
	* Sets the scan time parameter.
	* 
	* This function stores the desired value for scan_time.  In sensor_msgs::LaserScan, scan_time is defined as 
	* "time between scans [seconds]".  This value is not easily calculated from consquetive messages, and is thus
	* left to the user to set correctly.
	* 
	* @param scanTime The value to use for outgoing sensor_msgs::LaserScan.
	* 
	*/
	void DepthToLaserConverter::setScanTime(const float scanTime)
	{
	    _scanTime = scanTime;
	}

	/**
	* Sets the minimum and maximum range for the sensor_msgs::LaserScan.
	* 
	* range_min is used to determine how close of a value to allow through when multiple radii correspond to the same
	* angular increment.  range_max is used to set the output message.
	* 
	* @param rangeMin Minimum range to assign points to the laserscan, also minimum range to use points in the output scan.
	* @param rangeMax Maximum range to use points in the output scan.
	* 
	*/
	void DepthToLaserConverter::setRangeLimits(const float rangeMin, const float rangeMax)
	{
	    _rangeMin = rangeMin;
	    _rangeMax = rangeMax;
	}

	/**
	* Sets the number of image rows to use in the output LaserScan.
	* 
	* scan_height is the number of rows (pixels) to use in the output.  This will provide scan_height number of radii for each
	* angular increment.  The output scan will output the closest radius that is still not smaller than range_min.  This function
	* can be used to vertically compress obstacles into a single LaserScan.
	* 
	* @param scanHeight Number of pixels centered around the center of the image to compress into the LaserScan.
	* 
	*/
	void DepthToLaserConverter::setScanHeight(const int scanHeight)
	{
	    _scanHeight = scanHeight;
	}

	 /**
	* Sets the frame_id for the output LaserScan.
	* 
	* Output frame_id for the LaserScan.  Will probably NOT be the same frame_id as the depth image.
	* Example: For OpenNI cameras, this should be set to 'camera_depth_frame' while the camera uses 'camera_depth_optical_frame'.
	* 
	* @param outputFrameId Frame_id to use for the output sensor_msgs::LaserScan.
	* 
	*/
	void DepthToLaserConverter::setOutputFrame(const std::string outputFrameId)
	{
	    _outputFrameId = outputFrameId;
	}
    }
}
