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
	rapp::object::picture::Ptr image = pimpl->captureImage(camera_id, camera_resolution, encoding);
	return image;
}
rapp::object::qr_code_3d vision::qr_code_detection(rapp::object::picture::Ptr image, std::vector<std::vector<float>> robotToCameraMatrix, double camera_matrix[][3], float landmarkTheoreticalSize){
	rapp::object::qr_code_3d QRCodeStruct;
	try{
		QRCodeStruct = pimpl->qrCodeDetection(image, robotToCameraMatrix, camera_matrix, landmarkTheoreticalSize);//robotToCameraMatrix.matrix4x4.at(0)
	}
	catch(const std::runtime_error& re)
	{
		// speciffic handling for runtime_error
		std::cerr << "Runtime error: " << re.what() << std::endl;
	}catch(const std::exception& ex)
	{
		// speciffic handling for all exceptions extending std::exception, except
		// std::runtime_error which is handled explicitly
		std::cerr << "Error occurred: " << ex.what() << std::endl;
	}catch(...){
		std::cerr << "Unknown failure occured. Possible memory corruption" << std::endl;
	}
	return QRCodeStruct;
}
} // namespace rapp
} // namespace robot
