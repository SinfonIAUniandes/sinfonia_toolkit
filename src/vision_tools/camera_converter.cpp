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


#include "robot_toolkit/vision_tools/camera_converter.hpp"
#include <boost/foreach.hpp>
#define for_each BOOST_FOREACH
namespace enc = sensor_msgs::image_encodings;
namespace Sinfonia
{
    namespace Converter
    {
	CameraConverter::CameraConverter(const std::string& name, const float& frequency, const qi::SessionPtr& session, int cameraSource, int resolution, int colorSpace): 
	  BaseConverter(name, frequency, session)
	{	    
	    _pVideo = session->service("ALVideoDevice");
	    _pMemory = session->service("ALMemory");
	    _cameraSource = cameraSource;
	    std::vector<float> configs;
	    configs.push_back((float)resolution);
	    configs.push_back((float)frequency);
	    configs.push_back((float)colorSpace);
	    _compress = false;
	    _compressionFactor = 97;
	    setConfig(configs);
	    _serviceId = 0;
	    
	    
	}
	
	CameraConverter::~CameraConverter()
	{
	    if (!_handle.empty())
	    {
		_pVideo.call<qi::AnyValue>("unsubscribe", _handle);
		std::cout << BOLDYELLOW << "[" << ros::Time::now().toSec() << "] " << "Unsubscribe camera handle: " << _handle << std::endl;
		_handle.clear();
	    }
	}

	void CameraConverter::registerCallback(MessageAction::MessageAction action, Converter::CameraConverter::CallbackT callback)
	{
	    _callbacks[action] = callback;
	}
	void CameraConverter::callAll(const std::vector< MessageAction::MessageAction >& actions)
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
		_callbacks[action](_imageMsg, _cameraInfo);
	    }
	}
	void CameraConverter::reset()
	{
	    if (!_handle.empty())
	    {
		_pVideo.call<qi::AnyValue>("unsubscribe", _handle);
		_handle.clear();
	    }
	    _handle = _pVideo.call<std::string>("subscribeCamera", _name, _cameraSource, _resolution, _colorSpace, (int)_frequency);  
	}
	
	std::vector<float> CameraConverter::setParameters(std::vector<float> parameters)
	{
	    _pVideo.call<bool>("setCameraParameter", _handle, Helpers::VisionHelpers::kCameraBrightnessID, parameters[0]);
	    _pVideo.call<bool>("setCameraParameter", _handle, Helpers::VisionHelpers::kCameraContrastID, parameters[1]);
	    _pVideo.call<bool>("setCameraParameter", _handle, Helpers::VisionHelpers::kCameraSaturationID, parameters[2]);
	    _pVideo.call<bool>("setCameraParameter", _handle, Helpers::VisionHelpers::kCameraHueID, parameters[3]);
	    _pVideo.call<bool>("setCameraParameter", _handle, Helpers::VisionHelpers::kCameraHFlipID, parameters[4]);
	    _pVideo.call<bool>("setCameraParameter", _handle, Helpers::VisionHelpers::kCameraVFlipID, parameters[5]);
	    _pVideo.call<bool>("setCameraParameter", _handle, Helpers::VisionHelpers::kCameraAutoExpositionID, parameters[6]);
	    _pVideo.call<bool>("setCameraParameter", _handle, Helpers::VisionHelpers::kCameraAutoWhiteBalanceID, parameters[7]);
	    _pVideo.call<bool>("setCameraParameter", _handle, Helpers::VisionHelpers::kCameraAutoGainID, parameters[8]);
	    if(!parameters[8])
	    {
		_pVideo.call<bool>("setCameraParameter", _handle, Helpers::VisionHelpers::kCameraGainID, parameters[9]);
	    }
	    if(!parameters[6] )
	    {
		_pVideo.call<bool>("setCameraParameter", _handle, Helpers::VisionHelpers::kCameraExposureID, parameters[10]);
	    }
	    _pVideo.call<bool>("setCameraParameter", _handle, Helpers::VisionHelpers::kCameraSetDefaultParamsID, parameters[11]);
	    _pVideo.call<bool>("setCameraParameter", _handle, Helpers::VisionHelpers::kCameraAutoFocusID, parameters[18]);
	    if(parameters[19] == 0)
	    {
		_compress = false;
	    }
	    else
	    {
		_compress = true;
	    }
	    _compressionFactor = parameters[20];
	    return getParameters();
	}
	
	std::vector<float> CameraConverter::setAllParametersToDefault()
	{
	    _pVideo.call<bool>("setAllParametersToDefault", _cameraSource);
	    _compress = false;
	    _compressionFactor = 97;
	    return getParameters();
	}

	std::vector<float> CameraConverter::getParameters()
	{
	    std::vector<float> result;
	    result.push_back(_pVideo.call<int>("getCameraParameter", _handle, Helpers::VisionHelpers::kCameraBrightnessID));
	    result.push_back(_pVideo.call<int>("getCameraParameter", _handle, Helpers::VisionHelpers::kCameraContrastID));
	    result.push_back(_pVideo.call<int>("getCameraParameter", _handle, Helpers::VisionHelpers::kCameraSaturationID));
	    result.push_back(_pVideo.call<int>("getCameraParameter", _handle, Helpers::VisionHelpers::kCameraHueID));
	    result.push_back(_pVideo.call<int>("getCameraParameter", _handle, Helpers::VisionHelpers::kCameraHFlipID));
	    result.push_back(_pVideo.call<int>("getCameraParameter", _handle, Helpers::VisionHelpers::kCameraVFlipID));
	    result.push_back(_pVideo.call<int>("getCameraParameter", _handle, Helpers::VisionHelpers::kCameraAutoExpositionID));
	    result.push_back(_pVideo.call<int>("getCameraParameter", _handle, Helpers::VisionHelpers::kCameraAutoWhiteBalanceID));
	    result.push_back(_pVideo.call<int>("getCameraParameter", _handle, Helpers::VisionHelpers::kCameraAutoGainID));
	    result.push_back(_pVideo.call<int>("getCameraParameter", _handle, Helpers::VisionHelpers::kCameraGainID));
	    result.push_back(_pVideo.call<int>("getCameraParameter", _handle, Helpers::VisionHelpers::kCameraExposureID));
	    result.push_back(_pVideo.call<int>("getCameraParameter", _handle, Helpers::VisionHelpers::kCameraSetDefaultParamsID));
	    result.push_back(_pVideo.call<int>("getCameraParameter", _handle, Helpers::VisionHelpers::kCameraBlcRedID));
	    result.push_back(_pVideo.call<int>("getCameraParameter", _handle, Helpers::VisionHelpers::kCameraBlcGbID));
	    result.push_back(_pVideo.call<int>("getCameraParameter", _handle, Helpers::VisionHelpers::kCameraBlcBlueID));
	    result.push_back(_pVideo.call<int>("getCameraParameter", _handle, Helpers::VisionHelpers::kCameraResolutionID));
	    result.push_back(_pVideo.call<int>("getCameraParameter", _handle, Helpers::VisionHelpers::kCameraFrameRateID));
	    result.push_back(_pVideo.call<int>("getCameraParameter", _handle, Helpers::VisionHelpers::kCameraAverageLuminanceID));
	    result.push_back(_pVideo.call<int>("getCameraParameter", _handle, Helpers::VisionHelpers::kCameraAutoFocusID));
	    result.push_back((int)_compress);
	    result.push_back((int)_compressionFactor);
	    return result;
	}
	
	void CameraConverter::callCamera()
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
	    _imageMsg = cv_bridge::CvImage(std_msgs::Header(), _msgColorspace, cvImage).toImageMsg();
	    _imageMsg->header.frame_id = _msgFrameid;

	    _imageMsg->header.stamp = ros::Time::now();
	    _cameraInfo->header.stamp = _imageMsg->header.stamp;
	    if(_compress)
		compressImage();
	}
	
	void CameraConverter::compressImage()
	{
	    sensor_msgs::CompressedImage compressedImage;
	    compressedImage.header = _imageMsg->header;
	    compressedImage.format = _imageMsg->encoding;
	    std::vector<int> params;
	    params.resize(3, 0);
	    int bitDepth = enc::bitDepth(_imageMsg->encoding);
	    int numChannels = enc::numChannels(_imageMsg->encoding);
	    params[0] = CV_IMWRITE_JPEG_QUALITY;
	    params[1] = _compressionFactor;
	    
	    compressedImage.format += "; jpeg compressed";
	    if ((bitDepth == 8) || (bitDepth == 16))
	    {
		std::string targetFormat;
		if (enc::isColor(_imageMsg->encoding))
		{
		    targetFormat = "bgr8";
		    compressedImage.format += targetFormat;
		}

		try
		{
		    boost::shared_ptr<CameraConverter> trackedObject;
		    cv_bridge::CvImageConstPtr cvPtr = cv_bridge::toCvShare(*_imageMsg, trackedObject, targetFormat);
		
		    if (cv::imencode(".jpg", cvPtr->image, compressedImage.data, params))
		    {
			//float cRatio =  (float)compressedImage.data.size() / (float)(_imageMsg->data.size());
			//std::cout << "original size: " << _imageMsg->data.size() << " compressed size: " <<  compressedImage.data.size() << " compression factor: " << params[1] << std::endl;
		    }
		    else
		    {
			ROS_ERROR("cv::imencode (jpeg) failed on input image");
		    }
		}
		catch (cv_bridge::Exception& e)
		{
		    ROS_ERROR("%s", e.what());
		}
		catch (cv::Exception& e)
		{
		    ROS_ERROR("%s", e.what());
		}
		_imageMsg->data = compressedImage.data;
		_imageMsg->encoding = "compressed bgr8";
	    }
	    else
	    {
		ROS_ERROR("Compressed Image Transport - JPEG compression requires 8/16-bit color format (input format is: %s)", _imageMsg->encoding.c_str());
	    }
	}

	const sensor_msgs::CameraInfoPtr CameraConverter::getEmptyInfo()
	{
	    static const boost::shared_ptr<sensor_msgs::CameraInfo> camInfoMsg = boost::make_shared<sensor_msgs::CameraInfo>();
	    return camInfoMsg;
	}
	
	sensor_msgs::CameraInfoPtr CameraConverter::loadCameraInfo()
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
	    else if(_resolution == Helpers::VisionHelpers::kVGA && _cameraSource != Helpers::VisionHelpers::kBottomCamera )
	    {
		resolutionName = "kVGA";
	    }
	    else if(_resolution == Helpers::VisionHelpers::k4VGA && _cameraSource != Helpers::VisionHelpers::kBottomCamera)
	    {
		resolutionName = "k4VGA";
	    }
	    else if(_resolution == Helpers::VisionHelpers::k16VGA && _cameraSource != Helpers::VisionHelpers::kBottomCamera)
	    {
		resolutionName = "k16VGA";
	    }
	    else
	    {
		std::cout << BOLDYELLOW << "[" << ros::Time::now().toSec() << "] " << "Resolution not supported " << _cameraSource << " and res: " << _resolution << std::endl;
		return getEmptyInfo();
	    }
	    
	    boost::property_tree::ptree config;
	    std::string path;
	    static const boost::shared_ptr<sensor_msgs::CameraInfo> cameraInfoMessage = boost::make_shared<sensor_msgs::CameraInfo>();
	    if(_cameraSource == Helpers::VisionHelpers::kTopCamera )
	    {
		path = ros::package::getPath("robot_toolkit")+"/share/camera_info/top_camera_info.json";
		cameraInfoMessage->header.frame_id = "CameraTop_optical_frame";
	    }
	    else if(_cameraSource == Helpers::VisionHelpers::kBottomCamera)
	    {
		path = ros::package::getPath("robot_toolkit")+"/share/camera_info/bottom_camera_info.json";
		cameraInfoMessage->header.frame_id = "CameraBottom_optical_frame";
	    }
	    else
	    {
		path = ros::package::getPath("robot_toolkit")+"/share/camera_info/depth_camera_info.json";
		cameraInfoMessage->header.frame_id = "CameraTop_optical_frame";
	    }
	    
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
	    
	    cameraInfoMessage->width = config.get<int>(resolutionName + ".width", 0);
	    cameraInfoMessage->height = config.get<int>(resolutionName + ".height", 0);
	    cameraInfoMessage->distortion_model = config.get<std::string>(resolutionName + ".distortion_model", "None");
	    
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
	    
	    return cameraInfoMessage;
	}
	
	void CameraConverter::setConfig(std::vector<float> configs)
	{
	    _resolution = (int)configs[0];
	    setFrequency((int)configs[1]);
	    _colorSpace = (int)configs[2];

	    if( _colorSpace == Helpers::VisionHelpers::kYuvColorSpace || _colorSpace == Helpers::VisionHelpers::kyUvColorSpace || _colorSpace == Helpers::VisionHelpers::kyuVColorSpace || _colorSpace == Helpers::VisionHelpers::kRgbColorSpace ||
		_colorSpace == Helpers::VisionHelpers::krGbColorSpace || _colorSpace == Helpers::VisionHelpers::krgBColorSpace || _colorSpace == Helpers::VisionHelpers::kHsyColorSpace || _colorSpace == Helpers::VisionHelpers::khSyColorSpace ||
		_colorSpace == Helpers::VisionHelpers::khsYColorSpace )
	    {
		_msgColorspace = "mono8";
		_cvMatType = CV_8U;
	    }
	    else if( _colorSpace == Helpers::VisionHelpers::kYUV422ColorSpace || _colorSpace == Helpers::VisionHelpers::kYYCbCrColorSpace )
	    {
		_msgColorspace = "mono16";
		_cvMatType = CV_16UC2;
	    }
	    else if( _colorSpace == Helpers::VisionHelpers::kDepthColorSpace || _colorSpace == Helpers::VisionHelpers::kDistanceColorSpace || _colorSpace == Helpers::VisionHelpers::kRawDepthColorSpace )
	    {
		_msgColorspace = "16UC1";
		_cvMatType = CV_16U;
	    }
	    else if ( _colorSpace == Helpers::VisionHelpers::kXYZColorSpace )
	    {
		_msgColorspace = "rgb8";
		_cvMatType = CV_32FC3;
	    }
	    else
	    {
		_msgColorspace = "rgb8"; 
		_cvMatType = CV_8UC3;
	    }
	    _cameraInfo = loadCameraInfo();
	    if ( _cameraSource == Helpers::VisionHelpers::kTopCamera )
	    {
		_msgFrameid = "CameraTop_optical_frame";
	    }
	    else if (_cameraSource == Helpers::VisionHelpers::kBottomCamera )
	    {
		_msgFrameid = "CameraBottom_optical_frame";
	    }
	    else
	    {
		_msgFrameid = "CameraTop_optical_frame";
	    }
	}
	
	Helpers::VisionHelpers::NaoqiImage CameraConverter::fromAnyValueToNaoqiImage(qi::AnyValue& value)
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
    }
}
