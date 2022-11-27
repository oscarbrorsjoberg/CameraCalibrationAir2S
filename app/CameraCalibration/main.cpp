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


bool read_cmd_line(int argc, char *argv[],
		std::string &impath, std::string &conf, std::string &out, std::string &log_name)
{
	po::options_description opt("CameraCalibration options");

	opt.add_options()
		("help,h", "produce help message")
		("path,p", po::value<std::string>(&impath)->required(), "path to images")
		("conf,c", po::value<std::string>(&conf)->required(), "configuration file")
		("out,o", po::value<std::string>(&out)->required(), "out camera parameters in .yml")
		("log,l", po::value<std::string>(&log_name), "log information")
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
		std::string impath, out, conf, log; 

		if(!read_cmd_line(argc, argv, impath, conf, out, log)){
			return 0;
		}

		assert(fs::path(out).extension() == ".yml");
		assert(fs::path(conf).extension() == ".yml");
		assert(fs::path(log).extension() == ".txt");
		assert(fs::is_directory(impath));

		YAML::Node ymlConf = YAML::LoadFile(conf);

		Camera cam("test");	
		CalibrationConfig calibConf(ymlConf);

		/* todo : functionalize*/

		std::vector<std::vector<cv::Point2f>> allCrnrs;
		std::vector<cv::Point2f> foundPoints;
		std::vector<std::vector<cv::Point3f>> worldSpaceCornerPoints;

		/* todo check this! */
		cv::TermCriteria criteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 30, 0.001);
		bool found = false;
		int im_idx = 0, added = 0;

		std::vector<std::tuple<bool, std::string>> imageInformation;

		// collect points in image 
		for(const auto& en : fs::directory_iterator(impath)){

      if(!(en.path().extension() == ".JPG")){
        std::cout << "Only reading jpegs\n";
        continue;
      }

			cv::Mat image = cv::imread(en.path(), cv::IMREAD_GRAYSCALE);
			std::vector<cv::Point3f> worldCoords;

			if(im_idx == 0){
				/* cam.setPixWidth(image.size[1]/2); */
				/* cam.setPixHeight(image.size[0]/2); */

				cam.setPixWidth(image.size[1]);
				cam.setPixHeight(image.size[0]);

				// this is the 1" sensor in mm (air2s)
				/* cam.setSensorWidth(13.200); */
				/* cam.setSensorHeight(8.800); */
        
				cam.setSensorWidth(3.200);
				cam.setSensorHeight(2.400);

			}

			found = calibConf.findPoints(image, foundPoints);

			if(found){

				cv::Scalar scales = 
					estimateChessboardSharpness(image, calibConf.patternSize(), foundPoints); 

				std::cout << en.path() << std::endl;
				std::cout << scales << std::endl;

				std::cout << "image size " << image.size[1] << " "<< image.size[0] << std::endl;


				// is this always necessary??
				cv::cornerSubPix(image, foundPoints, cv::Size(11, 11), cv::Size(-1, -1), criteria);

				cv::drawChessboardCorners(image, calibConf.patternSize(), foundPoints, found);
				cv::imshow("Corners", image);
				cv::resizeWindow("Corners", 1020, 780);
				cv::setWindowProperty("Corners", cv::WINDOW_FULLSCREEN, 0.0);
				std::cout << "Press (a) for adding points, or press anything for not adding points\n";

				// show and choose	
				if((char)cv::waitKey(0) == 'a'){
					allCrnrs.push_back(foundPoints);
					createKnownBoardDim(calibConf.patternSize(),
							calibConf.dim(), 
							worldCoords);
					worldSpaceCornerPoints.push_back(worldCoords);
					added++;
				}
				else{
					std::cout << "Points not added!\n";
				}

			}
			std::cout << im_idx++ << std::endl;
		}


		if(allCrnrs.size() > 0){
			std::cout << "Starting calibration!" << std::endl;

			double rms = cam.calibrate(worldSpaceCornerPoints,
					allCrnrs, 
					calibConf);

			std::cout << "Calibration finished" << std::endl;


			std::cout << "=== Calibration result ===" << std::endl;
			std::cout << "== RMS:" << rms << std::endl;

			cam.write(out);
			cam.print();
			cam.dump_stats(log);
		}
		else{
			std::cout << "No points found!\n";
		}

	}
	catch(std::exception const & e) {
		std::cerr << e.what() << std::endl;
	}

	return 0;

}
