/**
 * @class Navigation
 * @brief Class which defines the interface for Robot navigation capabilities (movement, localization)
 * @date 10-August-2015
 * @author Wojciech Dudek <wojciechsbox@gmail.com>
 * @note This class uses pimpl pattern to make ABI as stable as possible when deploying new library versions
 */


#include <rapp-robots-api/vision/vision.hpp>

#include "VisionImpl.hpp"

#include <boost/filesystem.hpp>
#include <boost/property_tree/json_parser.hpp>

// Short alias for this namespace
namespace pt = boost::property_tree;
namespace fs = boost::filesystem;

std::string expand_user(std::string path) {
	if (not path.empty() and path[0] == '~') {
		assert(path.size() == 1 or path[1] == '/');  // or other error handling
		char const* home = getenv("HOME");
		if (home or (home = getenv("USERPROFILE"))) {
			path.replace(0, 1, home);
		}
		else {
			char const *hdrive = getenv("HOMEDRIVE");
			char const *hpath = getenv("HOMEPATH");
			assert(hdrive);  // or other error handling
			assert(hpath);
			path.replace(0, 1, std::string(hdrive) + hpath);
		}
	}
	return path;
}

struct camera_info {
	// camera matrix
	std::vector<float> K;

	// distortion coeffs
	std::vector<float> D;

	// projection matrix
	std::vector<float> P;
};

// read an array from json ptree
template <typename T>
std::vector<T> as_vector(pt::ptree const& pt, pt::ptree::key_type const& key) {
	std::vector<T> r;
	for (auto& item : pt.get_child(key))
		r.push_back(item.second.get_value<T>());
	return r;
}


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
vision::camera_info vision::load_camera_info(int camera_id) {
	vision::camera_info cam;
	std::string path = expand_user("~/.config/rapp_data/cam/") + std::to_string(camera_id) + ".json";
	if ( !boost::filesystem::exists( path ) )
	{
		std::cout << "Can't find calibration info for camera " << camera_id << ". Loading defaults." << std::endl;

		cam.K = {1373.92,       0, 655.54,
		                0, 1371.19, 462.62,
			        0,       0,   1};
		cam.D = {0, 0, 0, 0, 0};
		cam.P = cam.K;
	} else {
		pt::ptree tree;
		pt::read_json(path, tree);
		cam.K = as_vector<float>(tree, "K");
		cam.D = as_vector<float>(tree, "D");
		cam.P = as_vector<float>(tree, "P");
	}

	return cam;

}
} // namespace rapp
} // namespace robot
