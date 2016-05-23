/**
 * @class Vision
 * @brief Class which defines the interface for Robot Vision capabilities (movement, localization)
 * @date 10-August-2015
 * @author Wojciech Dudek <wojciechsbox@gmail.com>
 * @note This class uses pimpl pattern to make ABI as stable as possible when deploying new library versions
 */

#include "VisionImpl.hpp"

namespace rapp {
namespace robot {

VisionImpl::VisionImpl(int argc, char ** argv) {
    ros::init(argc, argv,"VisionImpl_library");
    n = new ros::NodeHandle();
}

VisionImpl::~VisionImpl() {
}

rapp::object::picture::Ptr VisionImpl::captureImage(int camera_id, int camera_resolution, const std::string & encoding) {
    if (!client_captureImage) {
        ROS_DEBUG("Invalid service client, creating new one...");
        double secs = ros::Time::now().toSec();
        client_captureImage = n->serviceClient<elektron_msgs::GetImage>("rapp_capture_image", true);
        double sec2 = ros::Time::now().toSec();
        ROS_DEBUG("Creating service client took %lf seconds", sec2-secs);
    } else {
        ROS_DEBUG("Service client valid.");
    }

    elektron_msgs::GetImage srv;
    srv.request.camera_id = camera_id;
    srv.request.resolution = camera_resolution;
    sensor_msgs::Image img;

    if (client_captureImage.call(srv)) {
        img = srv.response.frame;
        ROS_INFO("[Vision] - Image captured");
    } else {
        //Failed to call service rapp_get_image
        ROS_ERROR("[Vision] - Error calling service rapp_capture_image");
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
        if(camera_id >= 0 && camera_id <= 2) {
            cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
        } else if(camera_id >= 3 && camera_id <= 4) {
            cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_32FC1);
            cv_ptr = cv_bridge::cvtColor(cv_ptr, sensor_msgs::image_encodings::TYPE_16UC1);
        } else {
            throw new cv_bridge::Exception("bad camera_id");
        }
    } catch (cv_bridge::Exception e) {
        ROS_ERROR("[Vision] cv_bridge exception: %s", e.what());
        return std::make_shared<rapp::object::picture>("");
    }

    std::vector<unsigned char> bytes;
    std::vector<rapp::types::byte> rapp_bytes;
    cv::imencode(std::string(".") + encoding, cv_ptr->image, bytes);
    for (auto b : bytes) rapp_bytes.push_back(b);
    return std::make_shared<rapp::object::picture>(rapp_bytes);
}



} // namespace rapp
} // namespace robot
