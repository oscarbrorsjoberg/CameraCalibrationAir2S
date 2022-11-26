#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include <boost/program_options.hpp>

#include <yaml-cpp/yaml.h>

#include <iostream>
#include <sstream>
#include <fstream>
#include <filesystem>

#include "utils.hpp"
#include "camera.hpp"

namespace fs = std::filesystem;
namespace po = boost::program_options;

bool read_cmd_line(std::string &calib, 
									std::string &image)

{
	po::options_description opt("CameraUndistort");

	opt.add_options()
		("help,h", "produce help message")
		("calib,c", po::value<std::string>(&calib)->required(), "camera parameters")
		("image,i", po::value<std::string>(&image)->required(), "input image path")
		;

	po::variables_map vm;
	po::store(po::command_line_parser(argc, argv).options(opt).run(), vm);

	if(vm.count("help")){
		std::cout << opt << std::endl;
		return false;
	}

	po::notify(vm);
	return true;
}


int main(int argc, char *argv[])
{

	try{
		std::string im_path, camera_conf;
		


	}
	catch(std::exception const &e) {
		std::cerr << e.what() << std::endl;
		return 1;
	}

	return 0;
}
