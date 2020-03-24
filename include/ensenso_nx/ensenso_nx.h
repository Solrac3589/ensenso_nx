#ifndef ___ENSENSO_NX___ENSENSO_NX_H___
#define ___ENSENSO_NX___ENSENSO_NX_H___

#include <iostream>
#include <ros/ros.h>
#include <cmath>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <sensor_msgs/Image.h>
#include <ensenso_nx/HECalibrationAction.h>
#include <ensenso_camera_msgs/CalibrateHandEyeAction.h>
#include <ensenso_camera_msgs/ImagePoint.h>
#include <ensenso_camera_msgs/StereoCalibrationPattern.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // Needed for conversion from geometry_msgs to tf2::Transform

#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/PoseStamped.h>

#include <motion_server/FreeModeAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nxLib.h>
#include <ensenso_nx/conversion.h>
#include <ensenso_nx/calibration_pattern.h>
#include <thread>

namespace ensenso_nx
{

struct DeviceParams
{
	std::string serial_num;

	void print() const
	{
		std::cout << "\tSN: \t" << serial_num << std::endl;
	}
};

struct CaptureParams
{
	bool auto_exposure;
	unsigned int exposure_time; //in milliseconds TODO: check if uint is enough, or needs double
	int flex_view;
	bool dense_cloud; //Device::capture() returns a dense (ordered) point cloud if set to true

	void print() const
	{
		std::cout << "\tauto exposure [t/f]: \t" << auto_exposure << std::endl;
		std::cout << "\texposure [ms]: \t" << exposure_time << std::endl;
		std::cout << "\tdense cloud [t/f]: \t" << dense_cloud << std::endl;
	}
};

struct HECalParams
{
	double grid_spacing = 0.0;
	bool decode_data = true;
};

struct handEyeFeedback
{
	bool is_active = false;
	bool preempt_call = false;
	int  result = -1;
	ensenso_camera_msgs::CalibrateHandEyeFeedback feedback;
	bool send_feedback = false;
	const int SUCCEEDED = 0;
	const int PREEMPT = 1;
	const int ABORTED = 2;
};


class Device
{
protected:


	CaptureParams capture_params__;
	HECalParams he_cal_params__;
	DeviceParams device_params__;
	NxLibItem camera__;
	handEyeFeedback hand_eye_feedback;


	actionlib::SimpleActionClient<motion_server::FreeModeAction> free_mode__;

	NxLibItem nx_lib_root__;

	tf2_ros::Buffer tf2_buffer__;
	//std::shared_ptr<tf2_ros::TransformListener> tf_listener_ptr__;

	/** \brief Hardware set configuration
	 * Hardware set configuration
	 * \param _params: capture parameters
	**/
	void configureCapture();
	bool flexview_enabled__ = false ;
	tf2_ros::Buffer tfBuffer__;
	bool fixed_handeye__ = true; //for the moment, only fixed handeye
	std::string handEyeCalibrationPatternBuffer__;
	std::vector<tf2::Transform> handEyeCalibrationRobotPoses__;

	std::unique_ptr<std::thread> hand_eye_calibration_thread;
	std::string robotFrame__;
	std::string cameraFrame__;
	std::string wristFrame__;
	std::string targetFrame__;
	std::string linkFrame__;


public:
	/** \brief Constructor with serial number
	 * Constructor with serial number
	**/
	Device(const std::string & __serial_num);

	/** \brief Destructor
	 * Destructor
	**/
	~Device();

	/** \brief Set configuration for point cloud capture
	 * Set configuration for point cloud capture
	 * \param _params: capture parameters
	**/
	void configureCapture(const CaptureParams & __params);

	void configureHECal(const HECalParams & __params);

	/** \brief Set exposure in microseconds
	 * Set exposure in microseconds
	 * \param _exposure: exposure in milliseconds, a value of 0 indicates autoexposure
	**/
	//         void configureExposure(unsigned int _exposure);

	/** \brief Get a point cloud from the device
	 * Get a point cloud from the device
	 * \param _p_cloud: a point cloud where the capture data will be placed
	 * \return 1 if ok, -1 if not. TODO
	**/
	int capture(pcl::PointCloud<pcl::PointXYZ> & __p_cloud);
	int capture(pcl::PointCloud<pcl::PointXYZI> & __p_cloud);
	int capture(pcl::PointCloud<pcl::PointXYZRGB> & __p_cloud);
	ros::Time capture();
	std::vector<StereoCalibrationPattern> collectPattern(bool clearBuffer = false);
	//int HandsEyeCalibration(const ensenso_nx::HECalibrationGoalConstPtr &__goal, const ensenso_nx::HECalibrationResultConstPtr &__result);

	void HandsEyeCalibration(const ensenso_camera_msgs::CalibrateHandEyeGoalConstPtr &__goal, const ensenso_camera_msgs::CalibrateHandEyeResultConstPtr &__result);
	void HandsEyeCalibrationDetached(const ensenso_camera_msgs::CalibrateHandEyeGoalConstPtr &__goal, const ensenso_camera_msgs::CalibrateHandEyeResultConstPtr &__result);
	bool isCalibrationActive();
	void setCalibrationActive();
	void callCalibrationPreempt();
	bool requestFeedbackSend();
	void setFeedback(const bool __feedback);
	ensenso_camera_msgs::CalibrateHandEyeFeedback feedbackData();
	int getCalibrationResult();
	void resetCalibration();
	geometry_msgs::TransformStamped estimatePatternPose(ros::Time imageTimestamp, std::string const& targetFrame, bool latestPatternOnly);

	mutable std::map<std::string, geometry_msgs::TransformStamped> __transformationCache;
	bool isValid(tf2::Transform const& pose);
	bool isValid(tf2::Vector3 const& vector);
	bool isValid(geometry_msgs::Transform const& pose);
	void updateGlobalLink(ros::Time time, std::string frame = "", bool useCachedTransformation = false);
	tf2::Transform poseFromNxLib(NxLibItem const& node);
	geometry_msgs::TransformStamped poseFromNxLib(NxLibItem const& node, std::string const& parentFrame, std::string const& childFrame);
	void writePoseToNxLib(tf2::Transform const& pose, NxLibItem const& node);
	tf2::Transform fromMsg(geometry_msgs::Transform const& t);
	tf2::Transform fromMsg(geometry_msgs::Pose const& p);
	geometry_msgs::PoseStamped stampedPoseFromTransform(geometry_msgs::TransformStamped const& transform);
	geometry_msgs::Pose poseFromTransform(tf2::Transform const& transform);
	tf2::Transform fromStampedMessage(geometry_msgs::TransformStamped const& tStamped);

};

}

#endif
