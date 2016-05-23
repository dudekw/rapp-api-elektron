/**
 * @class Navigation
 * @brief Class which defines the interface for Robot navigation capabilities (movement, localization)
 * @date 10-August-2015
 * @author Wojciech Dudek <wojciechsbox@gmail.com>
 * @note This class uses pimpl pattern to make ABI as stable as possible when deploying new library versions
 */

#include <rapp-robots-api/vision/vision.hpp>

#include "VisionImpl.hpp"

namespace rapp {
	namespace robot {

		vision::vision(int argc, char ** argv ) {
			pimpl = new VisionImpl(argc, argv);
		}

		vision::~vision() {
			delete pimpl;
		}

		rapp::object::picture::Ptr vision::capture_image (int camera_id, int camera_resolution, const std::string & encoding) {
			if(camera_id < 0 || camera_id > 4) {
				ROS_ERROR("[Vision] - Invalid camera_id in capture_image - only values between 0 and 4 are supported");
			}
			rapp::object::picture::Ptr image = pimpl->captureImage(camera_id, camera_resolution, encoding);
			return image;
		}
	} // namespace rapp
} // namespace robot
