#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Imu.h>

#include <ST/CaptureSession.h>
#include <ST/CameraFrames.h>

namespace struct_core_ros {

class SCNodelet : public nodelet::Nodelet
{
private:
	ros::NodeHandle nh_;

	bool vis_enable_;
	image_transport::Publisher vis_pub_;
	ros::Publisher vis_caminfo_pub_;

	bool ir_enable_;
	image_transport::Publisher ir_pub_;
	ros::Publisher ir_caminfo_pub_;

	bool depth_enable_;
	image_transport::Publisher depth_image_pub_;
	ros::Publisher depth_caminfo_pub_;

	bool imu_enable_;
	ros::Publisher imu_pub_;
	sensor_msgs::Imu imu_;

    ST::CaptureSessionSettings sessionConfig_;
	ST::CaptureSession captureSession_;

	std::string frame_id_;

	ros::Timer heartbeat_timer_;
	bool start_streaming_ = false;

public:
	void onInit();
	static void eventCallback(void* userdata, ST::CaptureSession* session, const ST::CaptureSessionEventId& event);
	void captureSessionEventDidOccur(ST::CaptureSession* session, ST::CaptureSessionEventId event);
	static void outputCallback(void* userdata,  ST::CaptureSession* session, const  ST::CaptureSessionSample& sample);
	void captureSessionDidOutputSample(ST::CaptureSession* session, const  ST::CaptureSessionSample& sample);

	void handleIR(const ST::InfraredFrame &infraredFrame);
	void handleVis(const ST::ColorFrame &visFrame);
	void handleDepth(const ST::DepthFrame &depthFrame);
	void handleAccel(const ST::AccelerometerEvent &accelEvent);
	void handleGyro(const ST::GyroscopeEvent &gyroEvent);
	void sendIMU(double timestamp);
	void populateCamInfo(const ST::Intrinsics &intrinics, const std_msgs::Header header,
			sensor_msgs::CameraInfo &cam_info);
	void heartbeatCB(const ros::TimerEvent& event);
};

} // namespace struct_core
