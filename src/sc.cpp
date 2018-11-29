#include <stdio.h>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <functional>
#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <ST/CaptureSession.h>
#include <ST/CameraFrames.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define DEFAULT_FRAME_ID 	"sc_FLU"
#define LOG_PERIOD_S 		10
#define NODE_NAME 			"sc"

struct SessionDelegate : ST::CaptureSessionDelegate {
private:
    std::mutex lock;
    std::condition_variable cond;
    bool ready = false;
    bool done = false;
	ros::NodeHandle *nh_;
	std::string frame_id_;

	ST::CaptureSessionSettings *sessionConfig_;

	image_transport::Publisher vis_pub_;
	ros::Publisher vis_caminfo_pub_;

	image_transport::Publisher ir_pub_;
	ros::Publisher ir_caminfo_pub_;

	image_transport::Publisher depth_image_pub_;
	ros::Publisher depth_caminfo_pub_;

	ros::Publisher imu_pub_;
	sensor_msgs::Imu imu_;

public:
    SessionDelegate(ros::NodeHandle &nh, std::string &frame_id, ST::CaptureSessionSettings &sessionConfig)
    {
    	nh_ = &nh;
    	frame_id_ = frame_id;
    	sessionConfig_ = &sessionConfig;

		image_transport::ImageTransport it(*nh_);

		if(sessionConfig_->structureCore.depthEnabled)
		{
			// queue size is 1 because we only care about latest image
			depth_image_pub_ = it.advertise("depth/image_rect", 1);
			depth_caminfo_pub_ = nh_->advertise<sensor_msgs::CameraInfo>("depth/camera_info", 1);
		}

		if(sessionConfig_->structureCore.visibleEnabled)
		{
			// queue size is 1 because we only care about latest image
			vis_pub_ = it.advertise("visible/image_rect", 1);
			vis_caminfo_pub_ = nh_->advertise<sensor_msgs::CameraInfo>("visible/camera_info", 1);
		}

		if(sessionConfig_->structureCore.infraredEnabled)
		{
			// queue size is 1 because we only care about latest image
			ir_pub_ = it.advertise("ir/image_rect", 1);
			ir_caminfo_pub_ = nh_->advertise<sensor_msgs::CameraInfo>("ir/camera_info", 1);
		}

		if(sessionConfig_->structureCore.accelerometerEnabled || sessionConfig_->structureCore.gyroscopeEnabled)
		{
			imu_pub_ = nh_->advertise<sensor_msgs::Imu>("imu", 10);
		}
   }

    void captureSessionEventDidOccur(ST::CaptureSession *, ST::CaptureSessionEventId event) override {
        printf("Received capture session event %d (%s)\n", (int)event, ST::CaptureSessionSample::toString(event));
        switch (event) {
            case ST::CaptureSessionEventId::Ready: {
                std::unique_lock<std::mutex> u(lock);
                ready = true;
                cond.notify_all();
            } break;
            case ST::CaptureSessionEventId::Disconnected:
            case ST::CaptureSessionEventId::EndOfFile:
            case ST::CaptureSessionEventId::Error: {
                std::unique_lock<std::mutex> u(lock);
                done = true;
                cond.notify_all();
            } break;
            default:
                printf("Event %d unhandled\n", (int)event);
        }
    }

    void captureSessionDidOutputSample(ST::CaptureSession *, const ST::CaptureSessionSample& sample) {
        printf("Received capture session sample of type %d (%s)\n", (int)sample.type, ST::CaptureSessionSample::toString(sample.type));
        switch (sample.type) {
            case ST::CaptureSessionSample::Type::DepthFrame:
                printf("Depth frame: size %dx%d\n", sample.depthFrame.width(), sample.depthFrame.height());
                handleDepth(sample.depthFrame);
                break;
            case ST::CaptureSessionSample::Type::VisibleFrame:
                printf("Visible frame: size %dx%d\n", sample.visibleFrame.width(), sample.visibleFrame.height());
                handleVis(sample.visibleFrame);
                break;
            case ST::CaptureSessionSample::Type::InfraredFrame:
                printf("Infrared frame: size %dx%d\n", sample.infraredFrame.width(), sample.infraredFrame.height());
                handleIR(sample.infraredFrame);
                break;
            case ST::CaptureSessionSample::Type::SynchronizedFrames:
                printf("Synchronized frames: depth %dx%d visible %dx%d infrared %dx%d\n", sample.depthFrame.width(), sample.depthFrame.height(), sample.visibleFrame.width(), sample.visibleFrame.height(), sample.infraredFrame.width(), sample.infraredFrame.height());
                break;
            case ST::CaptureSessionSample::Type::AccelerometerEvent:
                printf("Accelerometer event: [% .5f % .5f % .5f]\n", sample.accelerometerEvent.acceleration().x, sample.accelerometerEvent.acceleration().y, sample.accelerometerEvent.acceleration().z);
                handleAccel(sample.accelerometerEvent);
                break;
            case ST::CaptureSessionSample::Type::GyroscopeEvent:
                printf("Gyroscope event: [% .5f % .5f % .5f]\n", sample.gyroscopeEvent.rotationRate().x, sample.gyroscopeEvent.rotationRate().y, sample.gyroscopeEvent.rotationRate().z);
                handleGyro(sample.gyroscopeEvent);
                break;
            default:
                printf("Sample type %d unhandled\n", (int)sample.type);
        }
    }

    void waitUntilReady() {
        std::unique_lock<std::mutex> u(lock);
        cond.wait(u, [this]() {
            return ready;
        });
    }

    void waitUntilDone() {
        std::unique_lock<std::mutex> u(lock);
        cond.wait(u, [this]() {
            return done;
        });
    }

	void handleIR(const ST::InfraredFrame &infraredFrame)
	{
		ROS_DEBUG_STREAM_THROTTLE(1.0, NODE_NAME << ": handleIR: w=" << infraredFrame.width() << ", h=" << infraredFrame.height());

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


	void handleVis(const ST::ColorFrame &visFrame)
	{
		ROS_DEBUG_STREAM_THROTTLE(1.0, NODE_NAME << ": handleVis: w=" << visFrame.width() << ", h=" << visFrame.height());

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


	void handleDepth(const ST::DepthFrame &depthFrame)
	{
		ROS_DEBUG_STREAM_THROTTLE(1.0, NODE_NAME << ": handleDepth: w=" << depthFrame.width() << ", h=" << depthFrame.height());

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

	void sendIMU(double timestamp)
	{
		// TODO: convert timstamp to ROS time
		// header.stamp =  timestamp();
		imu_.header.frame_id = frame_id_;

		// TODO: fuse accel and gyro
		imu_pub_.publish(imu_);
	}

	void handleAccel(const ST::AccelerometerEvent &accelEvent)
	{
		ROS_DEBUG_STREAM_THROTTLE(1.0, NODE_NAME << ": handleAccel");

		imu_.linear_acceleration.x = accelEvent.acceleration().x;
		imu_.linear_acceleration.y = accelEvent.acceleration().y;
		imu_.linear_acceleration.z = accelEvent.acceleration().z;
		sendIMU(accelEvent.timestamp());
	}

	void handleGyro(const ST::GyroscopeEvent &gyroEvent)
	{
		ROS_DEBUG_STREAM_THROTTLE(1.0, NODE_NAME << ": handleGyro");

		imu_.angular_velocity.x = gyroEvent.rotationRate().x;
		imu_.angular_velocity.y = gyroEvent.rotationRate().y;
		imu_.angular_velocity.z = gyroEvent.rotationRate().z;
		sendIMU(gyroEvent.timestamp());
	}

	void populateCamInfo(const ST::Intrinsics &intrinics,
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
};


class SCNode
{

private:
	ros::NodeHandle nh_;

	bool vis_enable_;

	bool ir_enable_;

	bool depth_enable_;

	bool imu_enable_;

	SessionDelegate *delegate_ = nullptr;
	ST::CaptureSessionSettings sessionConfig_;
	ST::CaptureSession captureSession_;

	bool streaming_ = false;

public:

	SCNode()  : nh_("~")
	{

		ros::param::param<bool>("~vis_enable", vis_enable_, false);
		ROS_INFO_STREAM(NODE_NAME << ": vis_enable = " << vis_enable_);

		ros::param::param<bool>("~ir_enable", ir_enable_, false);
		ROS_INFO_STREAM(NODE_NAME << ": ir_enable = " << ir_enable_);

		ros::param::param<bool>("~depth_enable", depth_enable_, false);
		ROS_INFO_STREAM(NODE_NAME << ": depth_enable = " << depth_enable_);

		ros::param::param<bool>("~imu_enable", imu_enable_, false);
		ROS_INFO_STREAM(NODE_NAME << ": imu_enable = " << imu_enable_);

		std::string frame_id;
		ros::param::param<std::string>("~frame_id", frame_id, DEFAULT_FRAME_ID);
		ROS_INFO_STREAM(NODE_NAME << ": frame_id = " << frame_id);

		ST::CaptureSessionSettings::StructureCoreSettings scConfig;

		scConfig.depthEnabled = depth_enable_;
		scConfig.infraredEnabled = ir_enable_;
		scConfig.visibleEnabled = vis_enable_;
		scConfig.accelerometerEnabled = imu_enable_;
		scConfig.gyroscopeEnabled = imu_enable_;

		scConfig.depthResolution = ST::StructureCoreDepthResolution::VGA;
		scConfig.visibleResolution = ST::StructureCoreVisibleResolution::Default;
		scConfig.infraredMode = ST::StructureCoreInfraredMode::LeftCameraOnly;
		scConfig.infraredResolution = ST::StructureCoreInfraredResolution::Default;

		sessionConfig_.source = ST::CaptureSessionSourceId::StructureCore;
		sessionConfig_.structureCore = scConfig;

		delegate_ = new SessionDelegate(nh_, frame_id, sessionConfig_);
		captureSession_.setDelegate(delegate_);
		if(!captureSession_.startMonitoring(sessionConfig_))
		{
			// throw exception
		}

	}

	~SCNode()
	{
		if(delegate_ != nullptr)
		{
			delete delegate_;
		}
	}

	void process()
	{
		if(!streaming_)
		{
			delegate_->waitUntilReady();
			captureSession_.startStreaming();
			streaming_ = true;
		}
//		delegate.waitUntilDone();
//		session.stopStreaming();
	}
};


int main(int argc, char **argv) {
	ros::init(argc, argv, NODE_NAME);

	SCNode *node = new SCNode();

	ROS_INFO_STREAM(NODE_NAME << ": running...");
	ros::Rate r(60);

	while (ros::ok()) {
		node->process();
		ros::spinOnce();
		r.sleep();
	}

	delete node;

	return 0;
}
