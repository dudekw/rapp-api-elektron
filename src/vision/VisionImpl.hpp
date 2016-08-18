#include "ros/ros.h"
#include <iostream>

#include "std_msgs/String.h"

#include "elektron_msgs/GetImage.h"

#include <rapp-robots-api/vision/vision.hpp>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <zbar.h> // for QRcode detection - dynamic agent
namespace rapp {
namespace robot {

class VisionImpl {

public:

	VisionImpl (int argc, char **argv);
	~VisionImpl();
	
	ros::ServiceClient client_captureImage;


	ros::NodeHandle *n;
	
	rapp::object::picture::Ptr captureImage(int camera_id, int camera_resolution, const std::string & encoding); /*
	Input: 
		cameraId: camera identifier,
		cameraResolution: camera resolution.
	Output: image (e.g. stored RGB image)
	Description: This function captures an image frame from the robot’s camera. The resolution of the captured image is set to cameraResolution. The returned color image is a kBGRColorSpace. The frame rate of the camera is set to 15 fps.
	*/

rapp::object::qr_code_3d qrCodeDetection(rapp::object::picture::Ptr imgFrame, std::vector<std::vector<float>> robotToCameraMatrix, double camera_matrix[][3], float landmarkTheoreticalSize = 0.16f);/*
	double camera_matrix[3][3] - camera intrinsic matrix
	*/	
};	
	
} // namespace robot
} // namespace rapp
