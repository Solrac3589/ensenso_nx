#include <ensenso_nx/ensenso_nx.h>
#include <thread>


#define LOG_NXLIB_EXCEPTION(EXCEPTION)
namespace ensenso_nx
{

void ensensoExceptionHandling (const NxLibException &ex, std::string func_nam)
{
	PCL_ERROR ("%s: NxLib error %s (%d) occurred while accessing item %s.\n", func_nam.c_str (), ex.getErrorText ().c_str (), ex.getErrorCode (),
	ex.getItemPath().c_str ());
	if (ex.getErrorCode() == NxLibExecutionFailed)
	{
		NxLibCommand cmd ("");
		PCL_WARN ("\n%s\n", cmd.result ().asJson (true, 4, false).c_str ());
	}
}

Device::Device(const std::string & __serial_num):
	free_mode__("/motion_server/free_mode", true)
{
	std::cout << "EnsensoNx::Device: Opening camera ..." << std::endl;

	nxLibInitialize(true);

	// Create an object referencing the camera's tree item, for easier access:
	camera__ = nx_lib_root__[itmCameras][itmBySerialNo][__serial_num];
	if (!camera__.exists() || (camera__[itmType] != valStereo))
	{
		std::cout << "EnsensoNx::Device: Camera not found. Please connect a single stereo camera to your computer" << std::endl;
		return;
	}
	device_params__.serial_num = camera__[itmSerialNumber].asString();
	//tf_listener_ptr__.reset(new tf2_ros::TransformListener(tf2_buffer__));

	NxLibCommand open(cmdOpen);
	open.parameters()[itmCameras] = device_params__.serial_num;
	open.execute();
	free_mode__.waitForServer();
	std::cout << "EnsensoNx::Device: Camera open. SN: " << device_params__.serial_num << std::endl;
}

Device::~Device()
{
	std::cout << "EnsensoNx::Device: Closing camera ..." << std::endl;
	NxLibCommand close(cmdClose);
	close.parameters()[itmCameras] = device_params__.serial_num;
	close.execute();
	std::cout << "EnsensoNx::Device: Camera closed. SN: " << device_params__.serial_num << std::endl;
	nxLibFinalize();
}

void Device::configureCapture(const CaptureParams & __params)
{
	capture_params__.auto_exposure = __params.auto_exposure;
	capture_params__.exposure_time = __params.exposure_time;
	capture_params__.dense_cloud = __params.dense_cloud;
	capture_params__.flex_view = __params.flex_view;
	configureCapture();
}

void Device::configureHECal(const HECalParams & __params)
{
	he_cal_params__.decode_data = __params.decode_data;
	he_cal_params__.grid_spacing = __params.grid_spacing;

}

// void Device::configureExposure(unsigned int _exposure)
// {
//     if (_exposure == 0) //autoexposure case
//     {
//         capture_params_.auto_exposure_ = true;
//     }
//     else //manual exposure case
//     {
//         capture_params_.auto_exposure_ = false;
//         capture_params_.exposure_time_ = _exposure;
//     }
//
//     //call protected member to set the configuration to the camera
//     this->configureCapture();
// }

int Device::capture(pcl::PointCloud<pcl::PointXYZ> & _p_cloud)
{

	int ww, hh;
	float px;
	std::vector<float> raw_points;
	int nx_return_code;

	NxLibCommand cam(cmdCapture);
	cam.parameters()[itmTimeout] = 25000;
	cam.execute();
	NxLibCommand (cmdComputeDisparityMap).execute();
	NxLibCommand (cmdComputePointMap).execute();

	// Get image dimensions
	camera__[itmImages][itmPointMap].getBinaryDataInfo(&ww, &hh, 0,0,0,0);

	//Get 3D image raw data
	camera__[itmImages][itmPointMap].getBinaryData(&nx_return_code, raw_points, 0);

	//Move raw data to point cloud
	_p_cloud.width = (unsigned int)ww;
	_p_cloud.height = (unsigned int)hh;
	_p_cloud.resize(_p_cloud.width*_p_cloud.height);
	unsigned int kk = 0;
	for(unsigned int ii = 0; ii<_p_cloud.height; ii++ )
	{
		for(unsigned int jj = 0; jj<_p_cloud.width; jj++ )
		{
			px = raw_points[(ii*_p_cloud.width + jj)*3];
			if ( !std::isnan(px) )
			{
				_p_cloud.points.at(kk).x = px/1000.;
				_p_cloud.points.at(kk).y = raw_points[(ii*_p_cloud.width + jj)*3 + 1]/1000.;
				_p_cloud.points.at(kk).z = raw_points[(ii*_p_cloud.width + jj)*3 + 2]/1000.;
				kk++;
			}
			else //in case of nan, check dense_cloud_ to fill in the cloud or not
			{
				if (capture_params__.dense_cloud)
				{
					_p_cloud.points.at(kk).x = std::nan("");
					_p_cloud.points.at(kk).y = std::nan("");
					_p_cloud.points.at(kk).z = std::nan("");
					kk++;
				}
				else
				{
					//nothing to do , the point is lost
				}
			}
		}
	}

	//resize with number valid points. If _dense_cloud, just set the flag ordered to true
	_p_cloud.resize(kk);//checks if kk=ww*hh to set the cloud as ordered (width,height) or unordered (width=size,height=1)
	_p_cloud.is_dense = capture_params__.dense_cloud;

	//debug message
	//     std::cout << "Cloud capture: " << std::endl <<
	//                  "\treturn code: " << nx_return_code << std::endl <<
	//                  "\tnum points: " << raw_points.size()/3 << std::endl <<
	//                  "\twidth: " << ww << std::endl <<
	//                  "\theight: " << hh << std::endl <<
	//                  "\tvalid_points: " << kk << std::endl;

	//return success
	return 1;
}



int Device::capture(pcl::PointCloud<pcl::PointXYZI> & _p_cloud)
{

	int ww, hh,www,hhh;
	float px;
	std::vector<float> raw_points;
	std::vector<float> raw_img;
  
  //raw_img_r  is not used for the moment. The pointcloud is focused in left_lens by default, so right one is hard to use. Nevertheles, here it is!
	std::vector<std::vector<uint8_t>> raw_img_r;
	std::vector<std::vector<uint8_t>> raw_img_l;
	int nx_return_code;

	NxLibCommand (cmdComputeDisparityMap).execute();
	NxLibCommand (cmdComputePointMap).execute();

	//Get image dimensions
	camera__[itmImages][itmPointMap].getBinaryDataInfo(&ww, &hh, 0,0,0,0);

	//Get 3D image raw data
	camera__[itmImages][itmPointMap].getBinaryData(&nx_return_code, raw_points, 0);

	//Get 2D image raw data
	raw_img_r.clear();
	raw_img_l.clear();
	int photos_set = capture_params__.flex_view;
	photos_set = photos_set - (photos_set % 4);
	if (!flexview_enabled__)
	{
		photos_set = 1;
		raw_img_r.resize(1);
		raw_img_l.resize(1);
		camera__[itmImages][itmRectified][itmLeft].getBinaryDataInfo(&www, &hhh, 0,0,0,0);
		camera__[itmImages][itmRectified][itmLeft].getBinaryData(raw_img_l[0], 0);
		camera__[itmImages][itmRectified][itmRight].getBinaryData(raw_img_r[0], 0);
	}
	else
	{
		camera__[itmImages][itmRectified][0][itmLeft].getBinaryDataInfo(&www, &hhh, 0,0,0,0);
		raw_img_r.resize(photos_set);
		raw_img_l.resize(photos_set);
		for (int i = 0; i < photos_set; i++)
		{

			camera__[itmImages][itmRectified][i][itmLeft].getBinaryData(&nx_return_code, raw_img_l[i], 0);
			camera__[itmImages][itmRectified][i][itmRight].getBinaryData(&nx_return_code, raw_img_r[i], 0);

		}

	}
	raw_img.resize((unsigned int)ww*(unsigned int)hh);

	for (size_t i = 0; i < raw_img.size(); i++)
	{
		raw_img[i] = 0;

		for (size_t j = 0; j < raw_img_l.size(); j++)
		{

				raw_img[i] = raw_img[i]  + raw_img_l[j][i];



		}
		raw_img[i] = raw_img[i]/raw_img_l.size();

	}

	//Move raw data to point cloud
	_p_cloud.width = (unsigned int)ww;
	_p_cloud.height = (unsigned int)hh;
	_p_cloud.resize(_p_cloud.width*_p_cloud.height);
	unsigned int kk = 0;
	for(unsigned int ii = 0; ii<_p_cloud.height; ii++ )
	{
		for(unsigned int jj = 0; jj<_p_cloud.width; jj++ )
		{
			px = raw_points[(ii*_p_cloud.width + jj)*3];
			if ( !std::isnan(px) )
			{
				_p_cloud.points.at(kk).x = px/1000.;
				_p_cloud.points.at(kk).y = raw_points[(ii*_p_cloud.width + jj)*3 + 1]/1000.;
				_p_cloud.points.at(kk).z = raw_points[(ii*_p_cloud.width + jj)*3 + 2]/1000.;
				_p_cloud.points.at(kk).intensity = raw_img[(ii*_p_cloud.width + jj)];//raw_img[kk];
				kk++;
			}
			else //in case of nan, check dense_cloud_ to fill in the cloud or not
			{
				if (capture_params__.dense_cloud)
				{
					//std::cout << "for some reason we are entering here at index " << kk << std::endl;
					_p_cloud.points.at(kk).x = std::nan("");
					_p_cloud.points.at(kk).y = std::nan("");
					_p_cloud.points.at(kk).z = std::nan("");
					_p_cloud.points.at(kk).intensity = 0;
					kk++;
				}
				else
				{
					//nothing to do , the point is lost
				}
			}
		}
	}

	//resize with number valid points. If _dense_cloud, just set the flag ordered to true
	_p_cloud.resize(kk);//checks if kk=ww*hh to set the cloud as ordered (width,height) or unordered (width=size,height=1)
	_p_cloud.is_dense = capture_params__.dense_cloud;

	//debug message
	//     std::cout << "Cloud capture: " << std::endl <<
	//                  "\treturn code: " << nx_return_code << std::endl <<
	//                  "\tnum points: " << raw_points.size()/3 << std::endl <<
	//                  "\twidth: " << ww << std::endl <<
	//                  "\theight: " << hh << std::endl <<
	//                  "\tvalid_points: " << kk << std::endl;

	//return success
	return 1;
}

int Device::capture(pcl::PointCloud<pcl::PointXYZRGB> & _p_cloud)
{

	int ww, hh;
	float px;
	std::vector<float> raw_points;
	std::vector<int> raw_img;
	int nx_return_code;



		NxLibCommand(cmdCapture).execute();
		NxLibCommand (cmdComputeDisparityMap).execute();
		NxLibCommand (cmdComputePointMap).execute();




	//Get image dimensions
	camera__[itmImages][itmPointMap].getBinaryDataInfo(&ww, &hh, 0,0,0,0);

	//Get 3D image raw data
	camera__[itmImages][itmPointMap].getBinaryData(&nx_return_code, raw_points, 0);

	//Get 2D image raw data
	if (!flexview_enabled__)
	{
		camera__[itmImages][itmRaw][itmLeft].getBinaryData(&nx_return_code, raw_img, 0);
	}
	else
	{
		camera__[itmImages][itmRaw][0][itmLeft].getBinaryData(&nx_return_code, raw_img, 0);
	}

	//Move raw data to point cloud
	_p_cloud.width = (unsigned int)ww;
	_p_cloud.height = (unsigned int)hh;
	_p_cloud.resize(_p_cloud.width*_p_cloud.height);
	unsigned int kk = 0;
	for(unsigned int ii = 0; ii<_p_cloud.height; ii++ )
	{
		for(unsigned int jj = 0; jj<_p_cloud.width; jj++ )
		{
			px = raw_points[(ii*_p_cloud.width + jj)*3];
			if ( !std::isnan(px) )
			{
				_p_cloud.points.at(kk).x = px/1000.;
				_p_cloud.points.at(kk).y = raw_points[(ii*_p_cloud.width + jj)*3 + 1]/1000.;
				_p_cloud.points.at(kk).z = raw_points[(ii*_p_cloud.width + jj)*3 + 2]/1000.;
				_p_cloud.points.at(kk).r = raw_img[(ii*_p_cloud.width + jj)*3];
				_p_cloud.points.at(kk).g = raw_img[(ii*_p_cloud.width + jj)*3+1];
				_p_cloud.points.at(kk).b = raw_img[(ii*_p_cloud.width + jj)*3+2];
				kk++;
			}
			else //in case of nan, check dense_cloud_ to fill in the cloud or not
			{
				if (capture_params__.dense_cloud)
				{
					_p_cloud.points.at(kk).x = std::nan("");
					_p_cloud.points.at(kk).y = std::nan("");
					_p_cloud.points.at(kk).z = std::nan("");
					_p_cloud.points.at(kk).r = 0;
					_p_cloud.points.at(kk).g= 0;
					_p_cloud.points.at(kk).b= 0;
					kk++;
				}
				else
				{
					//nothing to do , the point is lost
				}
			}
		}
	}

	//resize with number valid points. If _dense_cloud, just set the flag ordered to true
	_p_cloud.resize(kk);//checks if kk=ww*hh to set the cloud as ordered (width,height) or unordered (width=size,height=1)
	_p_cloud.is_dense = capture_params__.dense_cloud;

	//debug message
	//     std::cout << "Cloud capture: " << std::endl <<
	//                  "\treturn code: " << nx_return_code << std::endl <<
	//                  "\tnum points: " << raw_points.size()/3 << std::endl <<
	//                  "\twidth: " << ww << std::endl <<
	//                  "\theight: " << hh << std::endl <<
	//                  "\tvalid_points: " << kk << std::endl;

	//return success
	return 1;
}

void Device::configureCapture()
{

	camera__[itmParameters][itmCapture][itmAutoExposure] = capture_params__.auto_exposure;
	camera__[itmParameters][itmCapture][itmExposure    ] = static_cast<double>(capture_params__.exposure_time); //TODO check if requires cast to double.


	if (capture_params__.flex_view < 2)
	{

		flexview_enabled__ = false;
		camera__[itmParameters][itmCapture][itmFlexView] = flexview_enabled__;

	}
	else
	{

		flexview_enabled__ = true;
		camera__[itmParameters][itmCapture][itmFlexView] = static_cast<int>(capture_params__.flex_view);
		//camera__[itmParameters][itmCapture][itmHdr] = true;
		//camera__[itmParameters][itmCapture][itmGainBoost] = true;

	}

	if (capture_params__.flex_view > 8)
			camera__[itmParameters][itmDisparityMap][itmStereoMatching][itmMethod] = "Correlation";


}


/*
int Device::HandsEyeCalibration(const ensenso_nx::HECalibrationGoalConstPtr &__goal, const ensenso_nx::HECalibrationResultConstPtr &__result)
{

	//MISSING CHECK IF A PATTERN IS OBTAINED OR NOT. IN CASE IT IS NOT OBTAINED, REPEAT.

	// will need to adapt this line to the size of the calibration pattern that you are using.
	// Discard any pattern observations that might already be in the pattern buffer.
	NxLibCommand(cmdDiscardPatterns).execute();
	NxLibCommand decode(cmdCollectPattern);
	motion_server::FreeModeGoal goal_free_mode;
	goal_free_mode.until_button_pressed.data = true;
	int nx_return_code;
	double timestamp;
	if (!he_cal_params__.decode_data)
	{
			camera__[itmParameters][itmPattern][itmGridSpacing] = he_cal_params__.grid_spacing;
	}
	else
	{
			decode.parameters()[itmDecodeData] = true;
			decode.execute();
	}



	// Turn off the camera's projector so that we can observe the calibration pattern.
	camera__[itmParameters][itmCapture][itmProjector] = false;
	camera__[itmParameters][itmCapture][itmFrontLight] = true;

	// You can adapt this number depending on how accurate you need the
	// calibration to be.

	NxLibItem transformations_link(itmTransformations);
	for (int i = 0; i < __goal->position_number; i++) {
			// Move your robot to a new position from which the pattern can be seen. It might be a good idea to
			free_mode__.sendGoal(goal_free_mode);
			std::cout << "Waiting to confirm new position" << std::endl;
			free_mode__.waitForResult();
			std::cout << "New position have been declared" << std::endl;

			//Make sure that the robot is not moving anymore. You might want to wait for a few seconds to avoid
			//any oscillations.
			sleep(2);

			// Observe the calibration pattern and store the observation in the pattern buffer.
			NxLibCommand capture(cmdCapture);
			capture.parameters()[itmCameras] = device_params__.serial_num;
			capture.parameters()[itmTimeout] = 25000;
			capture.execute();
			bool foundPattern = false;
			try {
				std::vector<std::vector<float>> matrix_t;
					NxLibCommand collectPattern(cmdCollectPattern);
					collectPattern.parameters()[itmCameras] = device_params__.serial_num;
					collectPattern.execute();

					collectPattern.result()[itmStereo][itmLeft][itmTransformation].getBinaryData(&nx_return_code,matrix_t,&timestamp);

					std::cout << "Matrix: " << std::endl;
					for (size_t i = 0; i< matrix_t.size(); i++)
					{
						for (size_t j = 0; j< matrix_t[i].size(); j++)
						{
							std::cout << matrix_t[i][j] << " ";
						}
						std::cout << std::endl;
					}
					foundPattern = true;
			} catch (NxLibException&) {}

			if (foundPattern) {
					// We actually found a pattern. Get the current pose of your robot (from which the pattern was
					// observed) and store it somewhere.
//					geometry_msgs::TransformStamped table_to_ensenso = tf2_buffer__.lookupTransform("table", "world", ros::Time(0), ros::Duration(0.5));
//					transformations_link[i][itmRotation][itmAngle] = ;
//					transformations_link[i][itmRotation][itmAxis] = ;
//					transformations_link[i][itmRotation][itmAxis] = ;
//					transformations_link[i][itmRotation][itmAxis] = ;
//					transformations_link[i][itmTranslation] = ;
//					transformations_link[i][itmTranslation][itmAxis] = ;
//					transformations_link[i][itmTranslation][itmAxis] = ;

					//eigen::Angleaxis!!!!!!!

			} else {
					// The calibration pattern could not be found in the camera image. When your robot poses are
					// selected randomly, you might want to choose a different one.
			}
	}

	// You can insert a recalibration here, as you already captured stereo patterns anyway. See here for a
	// code snippet that does a recalibration.

	// We collected enough patterns and can start the calibration.
	NxLibCommand calibrateHandEye(cmdCalibrateHandEye);
	calibrateHandEye.parameters()[itmSetup] = __goal->type;

	// At this point, you need to put your stored robot poses into the command's Transformations parameter.
	//calibrateHandEye.parameters()[itmTransformations] = transformations_link;

	// Start the calibration. Note that this might take a few minutes if you did a lot of pattern observations.
	//calibrateHandEye.execute();

	//calibrateHandEye[itmResult][itmResidual].getBinaryData(&nx_return_code, __result->score, 0);
	//std::string return_d;
	//double residual = calibrateHandEye.result()[itmResidual].asDouble();


	//std::cout << "residual" << residual << std::endl;

	//std::cout << std::endl;
*/
/*	NxLibItem transformation_link = calibrateHandEye.result()[itmLink];
	std::cout << "Link rotation angle " << transformation_link[itmRotation][itmAngle].asDouble() << std::endl;
	std::cout << "Link rotation x" << transformation_link[itmRotation][itmAxis][0].asDouble() << std::endl;
	std::cout << "Link rotation y" << transformation_link[itmRotation][itmAxis][1].asDouble() << std::endl;
	std::cout << "Link rotation z" << transformation_link[itmRotation][itmAxis][2].asDouble() << std::endl;

	std::cout << "Link translation x" << transformation_link[itmTranslation][0].asDouble() << std::endl;
	std::cout << "Link translation y" << transformation_link[itmTranslation][1].asDouble() << std::endl;
	std::cout << "Link translation z" << transformation_link[itmTranslation][2].asDouble() << std::endl;*/
//	for (size_t i = 0; i < ret.size(); i++)
//	{
//		std::cout << transforms[i]<< " ";

//	}
//	std::cout << std::endl;

	// Store the new calibration to the camera's EEPROM.
	/*
	NxLibCommand storeCalibration(cmdStoreCalibration);
	storeCalibration.parameters()[itmCameras] = device_params__.serial_num;
	storeCalibration.parameters()[itmLink] = true;
	storeCalibration.execute();
	*/
/*
	camera__[itmParameters][itmCapture][itmProjector] = true;
	camera__[itmParameters][itmCapture][itmFrontLight] = false;
	return 0;


}*/

ros::Time Device::capture()
{
	ROS_DEBUG("Capturing an image...");

	NxLibCommand capture(cmdCapture, device_params__.serial_num);
	capture.parameters()[itmCameras] = device_params__.serial_num;

	capture.parameters()[itmTimeout] = 25000;
	capture.execute();

	NxLibItem imageNode = camera__[itmImages][itmRaw];
	imageNode = imageNode[itmLeft];

/*	if (isFileCamera)
	{
		// This workaround is needed, because the timestamp of captures from file cameras will not change over time. When
		// looking up the current tf tree, this will result in errors, because the time of the original timestamp is
		// requested, which lies in the past (and most often longer than the tfBuffer will store the transform!)
	return ros::Time::now();
	}

	return timestampFromNxLibNode(imageNode);
*/
	return ros::Time::now();
}

std::vector<StereoCalibrationPattern> Device::collectPattern(bool clearBuffer)
{
	if (clearBuffer)
	{
		NxLibCommand(cmdDiscardPatterns, device_params__.serial_num).execute();
	}

	NxLibCommand collectPattern(cmdCollectPattern, device_params__.serial_num);
	collectPattern.parameters()[itmCameras] = device_params__.serial_num;
	collectPattern.parameters()[itmDecodeData] = true;
	collectPattern.parameters()[itmFilter][itmCameras] = device_params__.serial_num;
	bool useModel = true;
	collectPattern.parameters()[itmFilter][itmUseModel] = useModel;
	collectPattern.parameters()[itmFilter][itmType] = valStatic;
	collectPattern.parameters()[itmFilter][itmValue] = true;
	try
	{
		collectPattern.execute();
	}
	catch (NxLibException& e)
	{
		if (e.getErrorCode() == NxLibExecutionFailed)
		{
			if (collectPattern.result()[itmErrorSymbol] == errPatternNotFound ||
			collectPattern.result()[itmErrorSymbol] == errPatternNotDecodable)
			{
				return {};
			}
		}
		throw;
	}

	if (!collectPattern.result()[itmStereo].exists())
	{
	// We did find patterns, but only in one of the cameras.
		return {};
	}

	std::vector<StereoCalibrationPattern> result;

///  result.resize(collectPattern.result()[itmStereo].count());

	for (int i = 0; i < collectPattern.result()[itmStereo].count(); i++)
	{
//	result[i].thickness = collectPattern.result()[itmStereo][i][itmThickness].asDouble();
//	result[i].grid_spacing = collectPattern.result()[itmStereo][i][itmGridSpacing].asDouble();
//	result[i].grid_size_x = collectPattern.result()[itmStereo][i][itmGridSize][0].asDouble();
//	result[i].grid_size_y = collectPattern.result()[itmStereo][i][itmGridSize][1].asDouble();
	result.emplace_back(collectPattern.result()[itmStereo][i]);
	}


	// Extract the pattern's image points from the result.
	NxLibItem pattern = collectPattern.result()[itmPatterns][0][device_params__.serial_num];
	if (pattern[itmLeft].count() > 1 || pattern[itmRight].count() > 1)
	{
		// We cannot tell which of the patterns in the two cameras belong together,
		// because that would need a comparison with the stereo model.
		return result;
	}

	for (size_t i = 0; i < result.size(); i++)
	{
		for (int j = 0; j < pattern[itmLeft][i][itmPoints].count(); j++)
		{
			NxLibItem pointNode = pattern[itmLeft][i][itmPoints][j];

			ensenso_camera_msgs::ImagePoint point;
			point.x = pointNode[0].asDouble();
			point.y = pointNode[1].asDouble();
			result.at(i).leftPoints.push_back(point);
		}
		for (int j = 0; j < pattern[itmRight][i][itmPoints].count(); j++)
		{
			NxLibItem pointNode = pattern[itmRight][i][itmPoints][j];

			ensenso_camera_msgs::ImagePoint point;
			point.x = pointNode[0].asDouble();
			point.y = pointNode[1].asDouble();
			result.at(i).rightPoints.push_back(point);
		}
	}

	return result;
}

tf2::Transform Device::fromMsg(geometry_msgs::Transform const& t)
{
	tf2::Transform transform;
	tf2::Quaternion quat;
	tf2::convert(t.rotation, quat);
	transform.setRotation(quat);
	tf2::Vector3 trans;
	tf2::convert(t.translation, trans);
	transform.setOrigin(trans);

	return transform;
}

tf2::Transform Device::fromMsg(geometry_msgs::Pose const& p)
{
	tf2::Transform transform;
	tf2::Quaternion quat;
	tf2::convert(p.orientation, quat);
	transform.setRotation(quat);
	tf2::Vector3 trans;
	tf2::convert(p.position, trans);
	transform.setOrigin(trans);

	return transform;
}

bool Device::isValid(tf2::Transform const& pose)
{
	auto origin = pose.getOrigin();
	if (!isValid(origin))
	{
		return false;
	}

	auto rotation = pose.getRotation();
	if (std::isnan(rotation.getAngle()))
	{
		return false;
	}

	auto rotationAxis = rotation.getAxis();
	return isValid(rotationAxis);
}

bool Device::isValid(tf2::Vector3 const& vector)
{
	return (!std::isnan(vector.x()) && !std::isnan(vector.y()) && !std::isnan(vector.z()));
}

bool Device::isValid(geometry_msgs::Transform const& pose)
{
	return isValid(fromMsg(pose));
}

tf2::Transform Device::poseFromNxLib(NxLibItem const& node)
{
	tf2::Transform pose;
	tf2::Vector3 origin;
	origin.setX(node[itmTranslation][0].asDouble() / 1000);  // NxLib
															// transformation is
															// in millimeters, ROS
															// expects it to be in
															// meters.
	origin.setY(node[itmTranslation][1].asDouble() / 1000);
	origin.setZ(node[itmTranslation][2].asDouble() / 1000);
	pose.setOrigin(origin);

	tf2::Vector3 rotationAxis(node[itmRotation][itmAxis][0].asDouble(), node[itmRotation][itmAxis][1].asDouble(),
						node[itmRotation][itmAxis][2].asDouble());
	tf2::Quaternion rotation(rotationAxis, node[itmRotation][itmAngle].asDouble());
	pose.setRotation(rotation);

	return pose;
}

geometry_msgs::TransformStamped Device::poseFromNxLib(NxLibItem const& node, std::string const& parentFrame,
											  std::string const& childFrame)
{
	geometry_msgs::TransformStamped stampedTransform;
	stampedTransform.header.stamp = ros::Time::now();
	stampedTransform.header.frame_id = parentFrame;
	stampedTransform.child_frame_id = childFrame;

	tf2::Transform transform = poseFromNxLib(node);
	tf2::convert(transform, stampedTransform.transform);
	return stampedTransform;
}


void Device::writePoseToNxLib(tf2::Transform const& pose, NxLibItem const& node)
{
	// Initialize the node to be empty. This is necessary, because there is a bug in some versions of the NxLib that
	// overwrites the whole transformation node with an identity transformation as soon as a new node in /Links gets
	// created.
	if (node.path.find("ViewPose") == std::string::npos)
	{
		// The ensenso SDK 2.2.x has a structure locked ViewPose item in the global params. So it cannot be set to null.
		node.setNull();
	}

	if (isValid(pose))
	{
		auto origin = pose.getOrigin();
		node[itmTranslation][0] = origin.x() * 1000;  // ROS transformation is in
													  // meters, NxLib expects it to
													  // be in millimeters.
		node[itmTranslation][1] = origin.y() * 1000;
		node[itmTranslation][2] = origin.z() * 1000;

		auto rotation = pose.getRotation();
		node[itmRotation][itmAngle] = rotation.getAngle();

		auto rotationAxis = rotation.getAxis();
		node[itmRotation][itmAxis][0] = rotationAxis.x();
		node[itmRotation][itmAxis][1] = rotationAxis.y();
		node[itmRotation][itmAxis][2] = rotationAxis.z();
	}
	else
	{
		ROS_ERROR("Given is pose is not valid for writing to the NxLib. Using identity transform");
		// Use an identity transformation as a reasonable default value.
		node[itmTranslation][0] = 0;
		node[itmTranslation][1] = 0;
		node[itmTranslation][2] = 0;

		node[itmRotation][itmAngle] = 0;
		node[itmRotation][itmAxis][0] = 1;
		node[itmRotation][itmAxis][1] = 0;
		node[itmRotation][itmAxis][2] = 0;
	}
}

void Device::updateGlobalLink(ros::Time time, std::string frame, bool useCachedTransformation)
{
	if (frame.empty())
	{
		frame = targetFrame__;
	}

	// Transformation are represented in the NxLib as follows.
	// The camera's link node contains the calibration data from e.g. the hand
	// eye calibration. This is always used when it is present.
	// The transformation between the link frame and the target frame (in
	// which the data is returned) is fetched from TF and written to a global link node
	// of the NxLib.
	// The link in the camera node has to reference this global link, if it exists. (e.g. when the linkFrame
	// is different from the targetFrame)

	if (linkFrame__ == frame)
	{
		// The frame is the target frame already. So the camera does not need a reference to a global link.
		camera__[itmLink][itmTarget] = "";
		return;
	}

	camera__[itmLink][itmTarget] = targetFrame__ + "_" + device_params__.serial_num;

	// Update the transformation to the target frame in the NxLib according to
	// the current information from TF. Only if the link frame and target frame differs.
	geometry_msgs::TransformStamped transform;
	if (useCachedTransformation && __transformationCache.count(frame) != 0)
	{
		transform = __transformationCache[frame];
	}
	else
	{
		transform = tfBuffer__.lookupTransform(linkFrame__, frame, time, ros::Duration(3000));
		__transformationCache[frame] = transform;
	}
	tf2::Transform tfTrafo;
	tf2::convert(transform.transform, tfTrafo);
	NxLibItem()[itmLinks][targetFrame__ + "_" + device_params__.serial_num].setNull();
	NxLibItem()[itmLinks].setNull();
	writePoseToNxLib(tfTrafo, NxLibItem()[itmLinks][targetFrame__ + "_" + device_params__.serial_num]);
}

geometry_msgs::TransformStamped Device::estimatePatternPose(ros::Time imageTimestamp,
																std::string const& targetFrame,
																bool latestPatternOnly)
{
	updateGlobalLink(imageTimestamp, targetFrame);

	NxLibCommand estimatePatternPose(cmdEstimatePatternPose, device_params__.serial_num);
	estimatePatternPose.parameters()[itmType] = itmMonocular;
	if (latestPatternOnly)
	{
		estimatePatternPose.parameters()[itmAverage] = false;

		int patternCount = NxLibItem()[itmParameters][itmPatternCount].asInt();
		estimatePatternPose.parameters()[itmFilter][itmOr][0][itmAnd][0][itmType] = valIndex;
		estimatePatternPose.parameters()[itmFilter][itmOr][0][itmAnd][0][itmValue] = patternCount - 1;
	}
	else
	{
		estimatePatternPose.parameters()[itmAverage] = true;
	}
	estimatePatternPose.execute();

	ROS_ASSERT(estimatePatternPose.result()[itmPatterns].count() == 1);

	return poseFromNxLib(estimatePatternPose.result()[itmPatterns][0][itmPatternPose], cameraFrame__, targetFrame);
}

geometry_msgs::PoseStamped Device::stampedPoseFromTransform(geometry_msgs::TransformStamped const& transform)
{
	geometry_msgs::PoseStamped pose;

	pose.pose.position.x = transform.transform.translation.x;
	pose.pose.position.y = transform.transform.translation.y;
	pose.pose.position.z = transform.transform.translation.z;
	pose.pose.orientation = transform.transform.rotation;

	pose.header.stamp = transform.header.stamp;
	pose.header.frame_id = transform.header.frame_id;

	return pose;
}

geometry_msgs::Pose Device::poseFromTransform(tf2::Transform const& transform)
{
	geometry_msgs::Pose pose;
	tf2::convert(transform.getRotation(), pose.orientation);
	pose.position.x = transform.getOrigin().x();
	pose.position.y = transform.getOrigin().y();
	pose.position.z = transform.getOrigin().z();

	return pose;
}

tf2::Transform Device::fromStampedMessage(geometry_msgs::TransformStamped const& tStamped)
{
	tf2::Transform transform;
	tf2::Quaternion quat;
	tf2::convert(tStamped.transform.rotation, quat);
	transform.setRotation(quat);
	tf2::Vector3 trans;
	tf2::convert(tStamped.transform.translation, trans);
	transform.setOrigin(trans);

	return transform;
}


void Device::HandsEyeCalibration(const ensenso_camera_msgs::CalibrateHandEyeGoalConstPtr &__goal, const ensenso_camera_msgs::CalibrateHandEyeResultConstPtr &__result)
{
	//START_NXLIB_ACTION(CalibrateHandEye, calibrateHandEyeServer)

	ensenso_camera_msgs::CalibrateHandEyeResult result;
	result.command = __goal->command;

	if (__goal->command == __goal->RESET)
	{
		handEyeCalibrationPatternBuffer__.clear();
		handEyeCalibrationRobotPoses__.clear();
	}
	else if (__goal->command == __goal->CAPTURE_PATTERN)
	{
		if (robotFrame__.empty() || wristFrame__.empty())
		{
			result.error_message = "You need to specify a robot base and wrist frame to do a hand eye calibration!";
			ROS_ERROR("%s", result.error_message.c_str());
			hand_eye_feedback.result = hand_eye_feedback.ABORTED;
			hand_eye_feedback.is_active = false;
			return;
		}

		//loadParameterSet(__goal->parameter_set, projectorOff);
		camera__[itmParameters][itmCapture][itmProjector] = false;
		camera__[itmParameters][itmCapture][itmFrontLight] = true;
		ros::Time imageTimestamp = capture();

		//PREEMPT_ACTION_IF_REQUESTED
		// Load the pattern buffer that we remembered from the previous
		// calibration steps.
		if (!handEyeCalibrationPatternBuffer__.empty())
		{
			NxLibCommand setPatternBuffer(cmdSetPatternBuffer, device_params__.serial_num);
			setPatternBuffer.parameters()[itmPatterns] << handEyeCalibrationPatternBuffer__;
			setPatternBuffer.execute();
		}
		else
		{
			NxLibCommand(cmdDiscardPatterns, device_params__.serial_num).execute();
		}

		std::vector<StereoCalibrationPattern> patterns = collectPattern();
		if (patterns.empty())
		{
			result.found_pattern = false;
			hand_eye_feedback.result = hand_eye_feedback.SUCCEEDED;
			hand_eye_feedback.is_active = false;
			return;
		}
		if (patterns.size() > 1)
		{
			result.error_message = "Detected multiple calibration patterns during a hand eye calibration!";
			ROS_ERROR("%s", result.error_message.c_str());
			hand_eye_feedback.result = hand_eye_feedback.ABORTED;
			hand_eye_feedback.is_active = false;
			return;
		}

		//PREEMPT_ACTION_IF_REQUESTED

		result.found_pattern = true;
		patterns[0].writeToMessage(result.pattern);

		auto patternPose = estimatePatternPose(imageTimestamp, cameraFrame__, true);
		result.pattern_pose = stampedPoseFromTransform(patternPose).pose;

		//PREEMPT_ACTION_IF_REQUESTED

		geometry_msgs::TransformStamped robotPose;
		try
		{
			robotPose = tfBuffer__.lookupTransform(robotFrame__, wristFrame__, ros::Time(0));
		}
		catch (tf2::TransformException& e)
		{
			result.error_message = std::string("Could not look up the robot pose due to the TF error: ") + e.what();
			ROS_ERROR("%s", result.error_message.c_str());
			hand_eye_feedback.result = hand_eye_feedback.ABORTED;
			hand_eye_feedback.is_active = false;
			return;
		}

		// Remember the newly collected data for the next step.
		NxLibCommand getPatternBuffer(cmdGetPatternBuffer, device_params__.serial_num);
		getPatternBuffer.execute();
		handEyeCalibrationPatternBuffer__ = getPatternBuffer.result()[itmPatterns].asJson();

		handEyeCalibrationRobotPoses__.push_back(fromStampedMessage(robotPose));

		result.robot_pose = stampedPoseFromTransform(robotPose).pose;

	}
	else if (__goal->command == __goal->START_CALIBRATION)
	{
		if (handEyeCalibrationRobotPoses__.size() < 5)
		{
			result.error_message = "You need collect at least 5 patterns before starting a hand eye calibration!";
			ROS_ERROR("%s", result.error_message.c_str());
			hand_eye_feedback.result = hand_eye_feedback.ABORTED;
			hand_eye_feedback.is_active = false;
			return;
		}

		// Load the pattern observations.
		size_t numberOfPatterns = 0;
		NxLibCommand setPatternBuffer(cmdSetPatternBuffer, device_params__.serial_num);
		if (!__goal->pattern_observations.empty())
		{
			for (size_t i = 0; i < __goal->pattern_observations.size(); i++)
			{
				StereoCalibrationPattern pattern(__goal->pattern_observations[i]);
				NxLibItem patternNode = setPatternBuffer.parameters()[itmPatterns][i][device_params__.serial_num];
				pattern.writeToNxLib(patternNode[itmLeft][0]);
				pattern.writeToNxLib(patternNode[itmRight][0], true);
			}
		}
		else
		{
			setPatternBuffer.parameters()[itmPatterns] << handEyeCalibrationPatternBuffer__;
		}
		numberOfPatterns = setPatternBuffer.parameters()[itmPatterns].count();
		setPatternBuffer.execute();


		// Load the corresponding robot poses.
		auto robotPoses = handEyeCalibrationRobotPoses__;
		if (!__goal->robot_poses.empty())
		{
			robotPoses.clear();
			for (auto const& pose : __goal->robot_poses)
			{
				tf2::Transform tfPose = fromMsg(pose);
				robotPoses.push_back(tfPose);
			}
		}

		if (robotPoses.size() != numberOfPatterns)
		{
			result.error_message = "The number of pattern observations does not match the number of robot poses!";
			ROS_ERROR("%s", result.error_message.c_str());
			hand_eye_feedback.result = hand_eye_feedback.ABORTED;
			hand_eye_feedback.is_active = false;
			return;
		}

		// Load the initial guesses from the action goal.
		tf2::Transform link, patternPose;
		link = fromMsg(__goal->link);
		patternPose = fromMsg(__goal->pattern_pose);

		NxLibCommand calibrateHandEye(cmdCalibrateHandEye, device_params__.serial_num);
		calibrateHandEye.parameters()[itmSetup] = fixed_handeye__ ? valFixed : valMoving;
		// The target node will be reset anyway before we calculate data for the next time.
		calibrateHandEye.parameters()[itmTarget] = targetFrame__ + "_" + device_params__.serial_num;
		if (isValid(link))
		{
			writePoseToNxLib(link.inverse(), calibrateHandEye.parameters()[itmLink]);
		}
		if (isValid(patternPose))
		{
			writePoseToNxLib(patternPose, calibrateHandEye.parameters()[itmPatternPose]);
		}
		for (size_t i = 0; i < robotPoses.size(); i++)
		{
			writePoseToNxLib(robotPoses[i], calibrateHandEye.parameters()[itmTransformations][i]);
		}

		calibrateHandEye.execute(false);


		auto getCalibrationResidual = [](NxLibItem const& node)
		{  // NOLINT
			if (node[itmResidual].exists())
			{
				return node[itmResidual].asDouble();
			}
			// Compatibility with the SDK 2.0.
			return node[itmReprojectionError].asDouble();
		};

		ros::Rate waitingRate(5);

		while (!calibrateHandEye.finished())
		{
			if (calibrateHandEye.result()[itmProgress].exists())
			{
				hand_eye_feedback.feedback.number_of_iterations = calibrateHandEye.result()[itmProgress][itmIterations].asInt();
				hand_eye_feedback.feedback.residual = getCalibrationResidual(calibrateHandEye.result()[itmProgress]);
				setFeedback(true);
			}

			if (hand_eye_feedback.preempt_call)
			{
				NxLibCommand(cmdBreak, device_params__.serial_num).execute();
				hand_eye_feedback.result = hand_eye_feedback.PREEMPT;
				hand_eye_feedback.is_active = false;
				return;
			}
			waitingRate.sleep();
		}

		result.calibration_time = calibrateHandEye.result()[itmTime].asDouble() / 1000;  // NxLib time is in milliseconds, ROS
																   // expects time to be in seconds.
		result.number_of_iterations = calibrateHandEye.result()[itmIterations].asInt();
		result.residual = getCalibrationResidual(calibrateHandEye.result());

		result.link = poseFromTransform(poseFromNxLib(camera__[itmLink]).inverse());
		result.pattern_pose = poseFromTransform(poseFromNxLib(calibrateHandEye.result()[itmPatternPose]));

		if (__goal->write_calibration_to_eeprom)
		{
			// Save the new calibration link to the camera's EEPROM.
			NxLibCommand storeCalibration(cmdStoreCalibration, device_params__.serial_num);
			storeCalibration.parameters()[itmCameras] = device_params__.serial_num;
			storeCalibration.parameters()[itmLink] = true;
			storeCalibration.execute();
		}
	}

	// The target frame has changed. TODO
	//publishCurrentLinks();
	hand_eye_feedback.result = hand_eye_feedback.SUCCEEDED;
	hand_eye_feedback.is_active = false;
	return;
	//FINISH_NXLIB_ACTION(CalibrateHandEye)

}

void Device::HandsEyeCalibrationDetached(const ensenso_camera_msgs::CalibrateHandEyeGoalConstPtr &__goal, const ensenso_camera_msgs::CalibrateHandEyeResultConstPtr &__result)
{

	hand_eye_calibration_thread.reset( new std::thread(&Device::HandsEyeCalibration, this,  __goal, __result) );
	hand_eye_calibration_thread->detach();

}

bool Device::isCalibrationActive()
{

	return hand_eye_feedback.is_active;

}
void Device::setCalibrationActive()
{

	hand_eye_feedback.is_active = true;

}
void Device::callCalibrationPreempt()
{

	hand_eye_feedback.preempt_call = true;

}

int Device::getCalibrationResult()
{

	return hand_eye_feedback.result;

}

void Device::resetCalibration()
{

	hand_eye_calibration_thread.reset();
	hand_eye_feedback.result = -1;
	hand_eye_feedback.is_active = false;
	hand_eye_feedback.preempt_call = false;
	hand_eye_feedback.send_feedback = false;

}

bool Device::requestFeedbackSend()
{

	return hand_eye_feedback.send_feedback;

}

void Device::setFeedback(const bool __feedback)
{

	hand_eye_feedback.send_feedback = __feedback;

}

ensenso_camera_msgs::CalibrateHandEyeFeedback Device::feedbackData()
{

	return hand_eye_feedback.feedback;

}


}
