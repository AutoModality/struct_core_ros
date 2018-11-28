#include <struct_core_ros/sc.h>
#include <memory>

#include <functional>

#include <nodelet/nodelet.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define DEFAULT_FRAME_ID 	"sc_FLU"
#define LOG_PERIOD_S 		10

namespace struct_core_ros {

SCNodelet *sc_nodelet;


void SCNodelet::onInit()
{
	nh_ = getNodeHandle();
	sc_nodelet = this;

	image_transport::ImageTransport it(nh_);

    ros::param::param<bool>("~vis_enable", vis_enable_, false);
    ROS_INFO_STREAM(getName() << ": vis_enable = " << vis_enable_);

    ros::param::param<bool>("~ir_enable", ir_enable_, false);
    ROS_INFO_STREAM(getName() << ": ir_enable = " << ir_enable_);

    ros::param::param<bool>("~depth_enable", depth_enable_, false);
    ROS_INFO_STREAM(getName() << ": depth_enable = " << depth_enable_);

    ros::param::param<bool>("~imu_enable", imu_enable_, false);
    ROS_INFO_STREAM(getName() << ": imu_enable = " << imu_enable_);

    ros::param::param<std::string>("~frame_id", frame_id_, DEFAULT_FRAME_ID);
    ROS_INFO_STREAM(getName() << ": frame_id = " << frame_id_);

	ST::CaptureSessionSettings::StructureCoreSettings scConfig;

	scConfig.depthEnabled = depth_enable_;
	if(depth_enable_)
	{
		// queue size is 1 because we only care about latest image
		depth_image_pub_ = it.advertise("depth/image_rect", 1);
		depth_caminfo_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("depth/camera_info", 1);
	}

	scConfig.visibleEnabled = vis_enable_;
    if(vis_enable_)
    {
		// queue size is 1 because we only care about latest image
    	vis_pub_ = it.advertise("visible/image_rect", 1);
    	vis_caminfo_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("visible/camera_info", 1);
    }

	scConfig.infraredEnabled = ir_enable_;
    if(ir_enable_)
    {
		// queue size is 1 because we only care about latest image
    	ir_pub_ = it.advertise("ir/image_rect", 1);
    	ir_caminfo_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("ir/camera_info", 1);
    }

	scConfig.accelerometerEnabled = imu_enable_;
	scConfig.gyroscopeEnabled = imu_enable_;
    if(imu_enable_)
    {
		imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu", 10);
    }

	scConfig.depthResolution = ST::StructureCoreDepthResolution::VGA;
	scConfig.visibleResolution = ST::StructureCoreVisibleResolution::Default;
	scConfig.infraredMode = ST::StructureCoreInfraredMode::LeftCameraOnly;
	scConfig.infraredResolution = ST::StructureCoreInfraredResolution::Default;

	sessionConfig_.source = ST::CaptureSessionSourceId::StructureCore;
    sessionConfig_.structureCore = scConfig;

	captureSession_.setDelegate(nullptr);
	captureSession_.setEventCallback(&SCNodelet::eventCallback, this);
	captureSession_.setOutputCallback(&SCNodelet::outputCallback, this);
	captureSession_.startMonitoring(sessionConfig_);

	heartbeat_timer_ = nh_.createTimer(ros::Duration(1.0), &SCNodelet::heartbeatCB, this);
}

void SCNodelet::eventCallback(void* userdata, ST::CaptureSession* session, const ST::CaptureSessionEventId& event)
{
	reinterpret_cast<SCNodelet*>(userdata)->captureSessionEventDidOccur(session, event);
}

void SCNodelet::captureSessionEventDidOccur(ST::CaptureSession* session, ST::CaptureSessionEventId event)
{
	ROS_INFO_STREAM(getName() << ": event:" << static_cast<std::underlying_type<ST::CaptureSessionEventId>::type>(event));
	if (session != &captureSession_)
	{
		ROS_WARN_STREAM_THROTTLE(1.0, getName() << ": event: wrong session");
	}

	if(event == ST::CaptureSessionEventId::Ready)
	{
		ROS_INFO_STREAM(getName() << ": event: received ready");
		start_streaming_ = true;
	}
}

void SCNodelet::outputCallback(void* userdata, ST::CaptureSession* session, const ST::CaptureSessionSample& sample)
{
     reinterpret_cast<SCNodelet*>(userdata)->captureSessionDidOutputSample(session, sample);
}

void SCNodelet::captureSessionDidOutputSample(ST::CaptureSession* session, const ST::CaptureSessionSample& sample)
{
    ROS_DEBUG_STREAM_THROTTLE(1.0, getName() << ": data: " << static_cast<std::underlying_type<ST::CaptureSessionSample::Type>::type>(sample.type));

    if (session != &captureSession_)
	{
        ROS_WARN_STREAM_THROTTLE(1.0, getName() << ": data: wrong session");
	}

    switch (sample.type) {
		case ST::CaptureSessionSample::Type::InfraredFrame:
			handleIR(sample.infraredFrame);
			break;
		case ST::CaptureSessionSample::Type::VisibleFrame:
			// handleVis(sample.visibleFrame);
			break;
		case ST::CaptureSessionSample::Type::DepthFrame:
			handleDepth(sample.depthFrame);
			break;
		case ST::CaptureSessionSample::Type::AccelerometerEvent:
			handleAccel(sample.accelerometerEvent);
			break;
		case ST::CaptureSessionSample::Type::GyroscopeEvent:
			handleGyro(sample.gyroscopeEvent);
			break;
    }
}


void SCNodelet::handleIR(const ST::InfraredFrame &infraredFrame)
{
	ROS_DEBUG_STREAM_THROTTLE(1.0, getName() << ": handleIR: w=" << infraredFrame.width() << ", h=" << infraredFrame.height());

	cv::Mat img(infraredFrame.height(), infraredFrame.width(), CV_8UC1);

	const uint16_t* buf = infraredFrame.data();

	for(int y = 0; y < infraredFrame.height(); y++)
	{
		for(int x = 0; x < infraredFrame.width(); x++)
		{
			std::size_t pixelOffset = (y * infraredFrame.width()) + x;
			uint16_t rgbPixel = buf[pixelOffset] >> 2;
			img.at<uchar>(y, x) = rgbPixel;
		}
	}

	std_msgs::Header header;
	header.frame_id = frame_id_;
	// TODO: convert timstamp to ROS time
	// header.stamp =  infraredFrame.timestamp();

	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "mono8", img).toImageMsg();
	ir_pub_.publish(msg);

	sensor_msgs::CameraInfo cam_info;
	populateCamInfo(infraredFrame.intrinsics(), header, cam_info);
	ir_caminfo_pub_.publish(cam_info);
}


void SCNodelet::handleVis(const ST::ColorFrame &visFrame)
{
	ROS_DEBUG_STREAM_THROTTLE(1.0, getName() << ": handleVis: w=" << visFrame.width() << ", h=" << visFrame.height());

	if(!visFrame.isValid())
	{
		return;
	}

	cv::Mat img(visFrame.height(), visFrame.width(), CV_8UC1);

	const uint8_t* buf = visFrame.rgbData();

	for(int y = 0; y < visFrame.height(); y++)
	{
		for(int x = 0; x < visFrame.width(); x++)
		{
			std::size_t pixelOffset = (y * visFrame.width()) + x;
			img.at<uchar>(y, x) = buf[pixelOffset];
		}
	}

	std_msgs::Header header;
	header.frame_id = frame_id_;
	// TODO: convert timstamp to ROS time
	// header.stamp =  infraredFrame.timestamp();

	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "mono8", img).toImageMsg();
	vis_pub_.publish(msg);

	sensor_msgs::CameraInfo cam_info;
	populateCamInfo(visFrame.intrinsics(), header, cam_info);
	vis_caminfo_pub_.publish(cam_info);
}


void SCNodelet::handleDepth(const ST::DepthFrame &depthFrame)
{
	ROS_DEBUG_STREAM_THROTTLE(1.0, getName() << ": handleDepth: w=" << depthFrame.width() << ", h=" << depthFrame.height());

	cv::Mat img(depthFrame.height(), depthFrame.width(), CV_32F);

	const float* buf = depthFrame.depthInMillimeters();

	for(int y = 0; y < depthFrame.height(); y++)
	{
		for(int x = 0; x < depthFrame.width(); x++)
		{
			std::size_t pixelOffset = (y * depthFrame.width()) + x;
			img.at<float>(y, x) = buf[pixelOffset];
		}
	}

	std_msgs::Header header;
	header.frame_id = frame_id_;
	// TODO: convert timstamp to ROS time
	// header.stamp =  infraredFrame.timestamp();

	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_32FC1, img).toImageMsg();
	depth_image_pub_.publish(msg);

	sensor_msgs::CameraInfo cam_info;
	populateCamInfo(depthFrame.intrinsics(), header, cam_info);
	depth_caminfo_pub_.publish(cam_info);
}

void SCNodelet::sendIMU(double timestamp)
{
	// TODO: convert timstamp to ROS time
	// header.stamp =  timestamp();
	imu_.header.frame_id = frame_id_;

	// TODO: fuse accel and gyro
	imu_pub_.publish(imu_);
}

void SCNodelet::handleAccel(const ST::AccelerometerEvent &accelEvent)
{
	ROS_DEBUG_STREAM_THROTTLE(1.0, getName() << ": handleAccel");

	imu_.linear_acceleration.x = accelEvent.acceleration().x;
	imu_.linear_acceleration.y = accelEvent.acceleration().y;
	imu_.linear_acceleration.z = accelEvent.acceleration().z;
	sendIMU(accelEvent.timestamp());
}

void SCNodelet::handleGyro(const ST::GyroscopeEvent &gyroEvent)
{
	ROS_DEBUG_STREAM_THROTTLE(1.0, getName() << ": handleGyro");

	imu_.angular_velocity.x = gyroEvent.rotationRate().x;
	imu_.angular_velocity.y = gyroEvent.rotationRate().y;
	imu_.angular_velocity.z = gyroEvent.rotationRate().z;
	sendIMU(gyroEvent.timestamp());
}


void SCNodelet::populateCamInfo(const ST::Intrinsics &intrinics,
		const std_msgs::Header header, sensor_msgs::CameraInfo &cam_info)
{
	cam_info.header = header;

	cam_info.height = intrinics.height;
	cam_info.width = intrinics.width;

	cam_info.distortion_model = "plumb_bob";

	cam_info.D.push_back(0);
	cam_info.D.push_back(0);
	cam_info.D.push_back(0);
	cam_info.D.push_back(0);
	cam_info.D.push_back(0);

	cam_info.K[0] = intrinics.fx;
	cam_info.K[1] = 0;
	cam_info.K[2] = intrinics.cx;
	cam_info.K[3] = 0;
	cam_info.K[4] = intrinics.fy;
	cam_info.K[5] = intrinics.cy;
	cam_info.K[6] = 0;
	cam_info.K[7] = 0;
	cam_info.K[8] = 1;

	cam_info.R[0] = 1;
	cam_info.R[1] = 0;
	cam_info.R[2] = 0;
	cam_info.R[3] = 0;
	cam_info.R[4] = 1;
	cam_info.R[5] = 0;
	cam_info.R[6] = 0;
	cam_info.R[7] = 0;
	cam_info.R[8] = 1;

	cam_info.P[0] = intrinics.fx;
	cam_info.P[1] = 0;
	cam_info.P[2] = intrinics.cx;
	cam_info.P[3] = 0;
	cam_info.P[4] = 0;
	cam_info.P[5] = intrinics.fy;
	cam_info.P[6] = intrinics.cy;
	cam_info.P[7] = 0;
	cam_info.P[8] = 0;
	cam_info.P[9] = 0;
	cam_info.P[10] = 1;
	cam_info.P[11] = 0;
}

void SCNodelet::heartbeatCB(const ros::TimerEvent& event)
{
	if(start_streaming_)
	{
		captureSession_.startStreaming();
		start_streaming_ = false;
	}
}

} //namespace struct_core_ros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(struct_core_ros::SCNodelet, nodelet::Nodelet);

